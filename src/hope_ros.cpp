#include <ros/ros.h>

#include <string.h>
#include <iostream>

#include <sensor_msgs/image_encodings.h>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "lib/plane_segment.h"
#include "lib/utilities.h"

//#define DEBUG

using namespace std;


int main(int argc, char **argv)
{
  // It is recommended to use launch file to start this node
  ros::init(argc, argv, "hope_ros");

  data_type type;

  type = REAL;
  ROS_INFO("Using real data.");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  string base_frame = "base_link"; // plane reference frame
  string cloud_topic;
  string camera_optical_frame_ = "vision_depth_optical_frame";

  float xy_resolution = 0.05; // In meter
  float z_resolution = 0.02; // In meter

  // Servo's max angle to rotate
  pnh.getParam("base_frame", base_frame);
  pnh.getParam("cloud_topic", cloud_topic);
  pnh.getParam("xy_resolution", xy_resolution);
  pnh.getParam("z_resolution", z_resolution);

  cout << "Using threshold: xy@" << xy_resolution 
       << " " << "z@" << z_resolution << endl;

  PlaneSegment hope(xy_resolution, z_resolution, base_frame, cloud_topic);
  PointCloud::Ptr src_cloud(new PointCloud); // Cloud input for all pipelines

  hope.setMode(type);
  while (ros::ok()) {
    // The src_cloud is actually not used here
    hope.getHorizontalPlanes(src_cloud);
  }

  return 0;
}
