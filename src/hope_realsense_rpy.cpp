//
// Created by smart on 2021/10/15.
//

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>

#include "hope/rs_motion.h"


using namespace std;


bool check_imu_is_supported()
{
  bool found_gyro = false;
  bool found_accel = false;
  rs2::context ctx;
  for (auto dev : ctx.query_devices()) {
    // The same device should support gyro and accel
    found_gyro = false;
    found_accel = false;
    for (const auto& sensor : dev.query_sensors()) {
      for (const auto& profile : sensor.get_stream_profiles()) {
        if (profile.stream_type() == RS2_STREAM_GYRO)
          found_gyro = true;
        if (profile.stream_type() == RS2_STREAM_ACCEL)
          found_accel = true;
      }
    }
    if (found_gyro && found_accel)
      break;
  }
  return found_gyro && found_accel;
}

void broadcastOrientation(const string& base_frame, const string& target_frame, hope::float3 theta){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = base_frame;
  transformStamped.child_frame_id = target_frame;
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  std::cerr << theta.x << " " << theta.y << " " << theta.z << std::endl;
  tf2::Quaternion q;
  q.setRPY(theta.roll(), theta.pitch(), theta.yaw());
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

int main(int argc, char * argv[]) try
{
  ros::init(argc, argv, "hope_realsense_orientation");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  string base_frame;
  string child_frame;
  pnh.getParam("base_frame", base_frame);
  pnh.getParam("child_frame", child_frame);
  if (base_frame.empty()) {
    ROS_ERROR("Base frame not given");
    return EXIT_FAILURE;
  }
  if (child_frame.empty()) {
    ROS_ERROR("Child frame not given");
    return EXIT_FAILURE;
  }

  // Before running the example, check that a device supporting IMU is connected
  if (!check_imu_is_supported()) {
    ROS_ERROR("Device supporting IMU not found");
    return EXIT_FAILURE;
  }

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;

  // Add streams of gyro and accelerometer to configuration
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

  // Declare object that handles camera pose calculations
  hope::RealSenseOrientationEstimator algo;

  // Start streaming with the given configuration;
  // Note that since we only allow IMU streams, only single frames are produced
  auto profile = pipe.start(cfg, [&](const rs2::frame& frame) {
    // Cast the frame that arrived to motion frame
    auto motion = frame.as<rs2::motion_frame>();
    // If casting succeeded and the arrived frame is from gyro stream
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
        motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
      // Get the timestamp of the current frame
      double ts = motion.get_timestamp();
      // Get gyro measures
      rs2_vector gyro_data = motion.get_motion_data();
      // Call function that computes the angle of motion based on the retrieved measures
      algo.processGyro(gyro_data, ts);
    }
    // If casting succeeded and the arrived frame is from accelerometer stream
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
        motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
      // Get accelerometer measures
      rs2_vector accel_data = motion.get_motion_data();
      // Call function that computes the angle of motion based on the retrieved measures
      algo.processAccel(accel_data);
    }
  });

  while (ros::ok()) {
    broadcastOrientation(base_frame, child_frame, algo.getTheta());
  }
  // Stop the pipeline
  pipe.stop();

  return EXIT_SUCCESS;
}
catch (const rs2::error & e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n " <<
            e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception& e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
