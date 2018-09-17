#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <math.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

// PCL-ROS
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Customized message
#include "lib/get_cloud.h"
#include "lib/plane_segment.h"
#include "lib/utilities.h"

//#define DEBUG

using namespace std;

// Publishers

/// Transform frame, only used with real time data
/// You may change the name based on your robot configuration
string base_frame_ = "base_link"; // world frame
string camera_optical_frame_ = "vision_depth_optical_frame";

int main(int argc, char **argv)
{
  // You need to run roscore to launch this program
  ros::init(argc, argv, "hope_ros");

  data_type type;

  type = REAL;
  ROS_INFO("Using real data.");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  float xy_resolution = 0.05; // In meter
  float z_resolution = 0.02; // In meter

  cout << "Using threshold: xy@" << xy_resolution 
       << " " << "z@" << z_resolution << endl;

  PlaneSegment hope(base_frame_, xy_resolution, z_resolution);
  PointCloud::Ptr src_cloud(new PointCloud); // Cloud input for all pipelines

  hope.setMode(type);
  while (ros::ok()) {
    // The src_cloud is actually not used here
    hope.getHorizontalPlanes(src_cloud);
  }

  return 0;
}
