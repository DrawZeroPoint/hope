#include <ros/ros.h>

#include <iostream>

#include <sensor_msgs/image_encodings.h>

#include "hope/plane_segment_ros.h"


using namespace std;


int main(int argc, char **argv)
{
  // It is recommended to use launch file to start this node
  ros::init(argc, argv, "hope_ros");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  float xy_resolution = 0.05; // In meter
  float z_resolution = 0.02; // In meter
  double max_depth = 10.; // In meter
  double min_depth = 0.; // In meter
  double max_height = 10; // In meter
  double min_height = -10; // In meter
  string base_frame = "base_link"; // plane reference frame
  string cloud_topic = "point_cloud";

  // Servo's max angle to rotate
  pnh.getParam("base_frame", base_frame);
  pnh.getParam("cloud_topic", cloud_topic);
  pnh.getParam("xy_resolution", xy_resolution);
  pnh.getParam("z_resolution", z_resolution);
  pnh.getParam("max_depth", max_depth);
  pnh.getParam("min_depth", min_depth);
  pnh.getParam("max_height", max_height);
  pnh.getParam("min_height", min_height);

  cout << "Using threshold: xy@" << xy_resolution
       << " " << "z@" << z_resolution << endl;

  hope::PlaneSegmentROS hope(
      nh, xy_resolution, z_resolution,
      min_depth, max_depth, min_height, max_height,
      base_frame, cloud_topic
  );

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
