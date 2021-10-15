//
// Created by smart on 2021/10/15.
//
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include "hope/rs_motion.h"


namespace hope {
  RealSenseOrientationEstimator::RealSenseOrientationEstimator():
      alpha_(0.98f),
      first_gyro_(true),
      first_accel_(true),
      last_ts_gyro_(0.) {
    theta_ = float3{};
  }

  // Function to calculate the change in angle of motion based on data from gyro
  void RealSenseOrientationEstimator::processGyro(rs2_vector gyro_data, double ts)
  {
    // On the first iteration, use only data from accelerometer to set the camera's initial position
    if (first_gyro_) {
      first_gyro_ = false;
      last_ts_gyro_ = ts;
      return;
    }
    // Holds the change in angle, as calculated from gyro
    float3 gyro_angle;

    // Initialize gyro_angle with data from gyro
    gyro_angle.x = gyro_data.x; // Pitch
    gyro_angle.y = gyro_data.y; // Yaw
    gyro_angle.z = gyro_data.z; // Roll

    // Compute the difference between arrival times of previous and current gyro frames
    double dt_gyro = (ts - last_ts_gyro_) / 1000.0;
    last_ts_gyro_ = ts;

    // Change in angle equals gyro measures * time passed since last measurement
    gyro_angle = gyro_angle * static_cast<float>(dt_gyro);

    // Apply the calculated change of angle to the current angle (theta)
    std::lock_guard<std::mutex> lock(theta_mtx_);
    theta_.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
  }

  void RealSenseOrientationEstimator::processAccel(rs2_vector accel_data)
  {
    // Holds the angle as calculated from accelerometer data
    float3 accel_angle;

    // Calculate rotation angle from accelerometer data
    accel_angle.z = atan2(accel_data.y, accel_data.z);
    accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

    // If it is the first iteration, set initial pose of camera according to accelerometer data
    // (note the different handling for Y axis)
    std::lock_guard<std::mutex> lock(theta_mtx_);
    if (first_accel_) {
      first_accel_ = false;
      theta_ = accel_angle;
      // Since we can't infer the angle around Y axis using accelerometer data,
      // we'll use PI as a convention for the initial pose
      theta_.y = PI_FL;
    } else {
      /*
      Apply Complementary Filter:
          - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
            that are steady over time, is used to cancel out drift.
          - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations
      */
      theta_.x = theta_.x * alpha_ + accel_angle.x * (1 - alpha_);
      theta_.z = theta_.z * alpha_ + accel_angle.z * (1 - alpha_);
    }
  }

  // Returns the current rotation angle
  float3 RealSenseOrientationEstimator::getTheta() {
    std::lock_guard<std::mutex> lock(theta_mtx_);
    return theta_;
  }
}


