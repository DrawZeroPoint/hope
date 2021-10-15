#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16MultiArray.h>

#include <geometry_msgs/PoseStamped.h>

#include <cmath>
#include <cstring>

#include "utilities.h"

using namespace std;

class Transform
{
public:
  Transform();

  bool getTransform(const string& base_frame, const string& header_frame);

  void doTransform(const Cloud_XYZRGB::Ptr& cloud_in, Cloud_XYZRGB::Ptr &cloud_out) const;
  
  void doTransform(const Cloud_XYZ::Ptr& cloud_in, Cloud_XYZ::Ptr &cloud_out) const;
  
  void doTransform(const Cloud_XYZRGB::Ptr& cloud_in, Cloud_XYZRGB::Ptr &cloud_out,
                   float roll, float pitch, float yaw);
  
  
  void doTransform(pcl::PointXYZ p_in, pcl::PointXYZ &p_out);
  
  void doTransform(const Cloud_XYZRGB::Ptr& cloud_in, Cloud_XYZRGB::Ptr &cloud_out, float tx, float ty, float tz,
                   float qx, float qy, float qz, float qw);
  
private:
  ros::NodeHandle nh_;
  // These declarations must be after the node initialization
  tf2_ros::Buffer tf_buffer_;
  // This is mandatory and should be declared before while loop  
  tf2_ros::TransformListener tf_listener_;
  
  geometry_msgs::TransformStamped tf_handle_;
};

#endif // TRANSFORM_H
