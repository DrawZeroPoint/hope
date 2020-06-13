#include <ros/ros.h>

#include <iostream>

#include <sensor_msgs/image_encodings.h>

#include "lib/plane_segment.h"

//#define DEBUG

using namespace std;


int main(int argc, char **argv)
{
  // It is recommended to use launch file to start this node
  ros::init(argc, argv, "hope_ros");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  string base_frame = "base_link"; // plane reference frame
  string cloud_topic = "point_cloud";

  float xy_resolution = 0.05; // In meter
  float z_resolution = 0.02; // In meter

  // Servo's max angle to rotate
  pnh.getParam("base_frame", base_frame);
  pnh.getParam("cloud_topic", cloud_topic);
  pnh.getParam("xy_resolution", xy_resolution);
  pnh.getParam("z_resolution", z_resolution);

  cout << "Using threshold: xy@" << xy_resolution 
       << " " << "z@" << z_resolution << endl;

  PlaneSegmentRT hope(xy_resolution, z_resolution, base_frame, cloud_topic);

  while (ros::ok()) {
    // The src_cloud is actually not used here
    hope.getHorizontalPlanes();
  }

  return 0;
}
