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
string base_frame_ = "mobile_base_link"; // world frame

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

  double x_dim = 0.4;
  double y_dim = 0.2;
  double z_min = -0.12;
  double z_max = 0.5;

  pnh.getParam("x_dim", x_dim);
  pnh.getParam("y_dim", y_dim);
  pnh.getParam("z_min", z_min);
  pnh.getParam("z_max", z_max);

  PlaneSegment hope(base_frame_, xy_resolution, z_resolution, nh, x_dim, y_dim, z_min, z_max);

  ros::Rate loop_rate(0.5);
  hope.setMode(type);
  while (ros::ok()) {
    // The src_cloud is actually not used here
    hope.getHorizontalPlanes();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
