//
// Created by smart on 2021/10/15.
//

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>

#include <iostream>

#include "hope/rs_motion.h"


using namespace std;

// Declare object that handles camera pose calculations
hope::RealSenseOrientationEstimator algo_;

void broadcastOrientation(const string& base_frame, const string& target_frame, hope::float3 theta){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = base_frame;
  transformStamped.child_frame_id = target_frame;
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  // The order is determined by experiments
  q.setRPY(-theta.x, -theta.z - PI_FL_2, theta.y - PI_FL);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

void gyroCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  // Get the timestamp of the current frame
  double ts = msg->header.stamp.toSec() * 1000.;  // ms
  // Get gyro measures
  rs2_vector gyro_data;
  gyro_data.x = static_cast<float>(msg->angular_velocity.x);
  gyro_data.y = static_cast<float>(msg->angular_velocity.y);
  gyro_data.z = static_cast<float>(msg->angular_velocity.z);
  // Call function that computes the angle of motion based on the retrieved measures
  algo_.processGyro(gyro_data, ts);
}

void accelCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  // Get accelerometer measures
  rs2_vector accel_data;
  accel_data.x = static_cast<float>(msg->linear_acceleration.x);
  accel_data.y = static_cast<float>(msg->linear_acceleration.y);
  accel_data.z = static_cast<float>(msg->linear_acceleration.z);
  // Call function that computes the angle of motion based on the retrieved measures
  algo_.processAccel(accel_data);
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "hope_realsense_rpy_ros");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  string base_frame;
  string child_frame;
  string gyro_topic;
  string accel_topic;
  double rate = 30.;
  pnh.getParam("base_frame", base_frame);
  pnh.getParam("child_frame", child_frame);
  pnh.getParam("gyro_topic", gyro_topic);
  pnh.getParam("accel_topic", accel_topic);
  pnh.getParam("rate", rate);
  if (base_frame.empty()) {
    ROS_ERROR("Base frame not given");
    return EXIT_FAILURE;
  }
  if (child_frame.empty()) {
    ROS_ERROR("Child frame not given");
    return EXIT_FAILURE;
  }
  if (gyro_topic.empty()) {
    ROS_ERROR("Gyro topic not given");
    return EXIT_FAILURE;
  }
  if (accel_topic.empty()) {
    ROS_ERROR("Accel topic not given");
    return EXIT_FAILURE;
  }

  ros::Subscriber gyro_sub = nh.subscribe<sensor_msgs::Imu>(gyro_topic, 1, gyroCallback);
  ros::Subscriber accel_sub = nh.subscribe<sensor_msgs::Imu>(accel_topic, 1, accelCallback);

  double loop_time = 1. / rate;  // in seconds
  while (ros::ok()) {
    ros::spinOnce();
    broadcastOrientation(base_frame, child_frame, algo_.getTheta());
    ros::Duration(loop_time).sleep();
  }
  return EXIT_SUCCESS;
}
