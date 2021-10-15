//
// Created by smart on 2021/10/15.
//

#ifndef SRC_RS_MOTION_H
#define SRC_RS_MOTION_H


#include <librealsense2/rs.hpp>
#include <mutex>
#include <cmath>
#include <cstring>
#include <iostream>


#define PI_FL  3.141592f
#define PI_FL_2  1.570796f

namespace hope {

  struct float3 {
    float x, y, z;

    float roll() const { return z; }
    float pitch() const { return x; }
    float yaw() const { return y; }

    float3 operator*(float t) const {
      return { x * t, y * t, z * t };
    }

    float3 operator-(float t) const {
      return { x - t, y - t, z - t };
    }

    void operator*=(float t) {
      x = x * t;
      y = y * t;
      z = z * t;
    }

    void operator=(float3 other) {
      x = other.x;
      y = other.y;
      z = other.z;
    }

    void add(float t1, float t2, float t3) {
      x += t1;
      y += t2;
      z += t3;
    }
  };

  class RealSenseOrientationEstimator {

  public:
    RealSenseOrientationEstimator();
    ~RealSenseOrientationEstimator() = default;

    void processGyro(rs2_vector gyro_data, double ts);
    void processAccel(rs2_vector accel_data);
    float3 getTheta();

  private:
    // theta_ is the angle of camera rotation in x, y and z components
    float3 theta_;
    std::mutex theta_mtx_;

    /* alpha indicates the part that gyro and accelerometer take in computation of theta;
     * higher alpha gives more weight to gyro, but too high values cause drift;
     * lower alpha gives more weight to accelerometer, which is more sensitive to disturbances
     */
    float alpha_;

    bool first_gyro_;
    bool first_accel_;

    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro_;

  };
}

#endif //SRC_RS_MOTION_H
