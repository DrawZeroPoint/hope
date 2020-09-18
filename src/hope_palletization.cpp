#include <ros/ros.h>

#include <iostream>

#include <sensor_msgs/image_encodings.h>

#include "lib/palletization.h"

//#define DEBUG

using namespace std;


int main(int argc, char **argv)
{
  // It is recommended to use launch file to start this node
  ros::init(argc, argv, "hope_palletization");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  float xy_resolution = 0.05; // In meter
  float z_resolution = 0.03; // In meter
  string base_frame = "base_link"; // plane reference frame

  // Servo's max angle to rotate
  pnh.getParam("base_frame", base_frame);
  pnh.getParam("xy_resolution", xy_resolution);
  pnh.getParam("z_resolution", z_resolution);

  cout << "Using threshold: xy@" << xy_resolution 
       << " " << "z@" << z_resolution << endl;

  Palletization hope(nh, base_frame, xy_resolution, z_resolution);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
