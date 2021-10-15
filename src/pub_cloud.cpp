#include <ros/ros.h>

#include <iostream>

#include <sensor_msgs/image_encodings.h>

// PCL
#include <pcl/filters/extract_indices.h>

#include "hope/fetch_rgbd.h"
#include "hope/get_cloud.h"
#include "hope/utilities.h"

//#define DEBUG

using namespace std;
using namespace hope;


int main(int argc, char **argv)
{
  // It is recommended to use launch file to start this node
  ros::init(argc, argv, "hope_pub_cloud");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  string camera_frame; // frame of the depth camera
  string depth_topic; // input depth image topic
  string cloud_topic; // output cloud topic

  float fx;
  float fy;
  float cx;
  float cy;
  float min_depth;
  float max_depth;

  // Servo's max angle to rotate
  pnh.getParam("camera_frame", camera_frame);

  pnh.getParam("depth_topic", depth_topic);
  pnh.getParam("cloud_topic", cloud_topic);

  pnh.getParam("fx", fx);
  pnh.getParam("fy", fy);
  pnh.getParam("cx", cx);
  pnh.getParam("cy", cy);

  pnh.getParam("min_depth", min_depth);
  pnh.getParam("max_depth", max_depth);

  cerr << "camera frame: " << camera_frame << endl
       << "depth_topic " << depth_topic << endl
       << "cloud_topic " << cloud_topic << endl
       << "fx " << fx << endl
       << "fy " << fy << endl
       << "cx " << cx << endl
       << "cy " << cy << endl
       << "min_depth " << min_depth << endl
       << "max_depth " << max_depth << endl;

  FetchRGBD fetcher(depth_topic);
  Cloud_XYZ::Ptr src_cloud(new Cloud_XYZ);

  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1);

  while (ros::ok()) {
    cv_bridge::CvImagePtr depth;
    fetcher.fetchDepth(depth);

    GetCloud::getMonoCloud(depth->image, fx, fy, cx, cy, max_depth, min_depth, src_cloud);
    Utilities::publishCloud<Cloud_XYZ::Ptr>(src_cloud, pub_cloud, camera_frame);
  }

  return 0;
}
