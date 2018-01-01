#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <math.h>
#include <string.h>

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

#include <image_geometry/pinhole_camera_model.h>

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

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom message
#include "lib/get_cloud.h"
#include "lib/plane_segment.h"
#include "lib/utilities.h"

using namespace std;
using namespace cv;

enum dataset{DEFAULT, TUM};

// Publishers

// Status
bool use_real_data_ = false; // If using real-time image and imu data 

// Marker type
uint32_t shape = visualization_msgs::Marker::ARROW;

// Transform frame, only used with real time data
// You may change the name based on your robot configuration
string base_frame_ = "base_link"; // world frame
string camera_optical_frame_ = "vision_depth_optical_frame";

// Camera orientation params, only used in testing with benchmark data
float roll_angle_;
float pitch_angle_;

float tx_, ty_, tz_, qx_, qy_, qz_, qw_;

//void publishMarker(geometry_msgs::PoseStamped pose)
//{
//  visualization_msgs::Marker marker;
//  // Set the frame ID and timestamp. See the TF tutorials for information on these.
//  marker.header.frame_id = pose.header.frame_id;
//  marker.header.stamp = pose.header.stamp;

//  // Set the namespace and id for this marker. This serves to create a unique ID
//  // Any marker sent with the same namespace and id will overwrite the old one
//  marker.ns = "put";
//  marker.id = 0;

//  marker.type = shape;
//  marker.action = visualization_msgs::Marker::ADD;

//  // Set the pose of the marker, which is a full 6DOF pose
//  // relative to the frame/time specified in the header
//  marker.points.resize(2);
//  // The point at index 0 is assumed to be the start point,
//  // and the point at index 1 is assumed to be the end.
//  marker.points[0].x = pose.pose.position.x;
//  marker.points[0].y = pose.pose.position.y;
//  marker.points[0].z = pose.pose.position.z;

//  marker.points[1].x = pose.pose.position.x;
//  marker.points[1].y = pose.pose.position.y;
//  marker.points[1].z = pose.pose.position.z + 0.15; // arrow height = 0.15m

//  // scale.x is the shaft diameter, and scale.y is the head diameter.
//  // If scale.z is not zero, it specifies the head length.
//  // Set the scale of the marker -- 1x1x1 here means 1m on a side
//  marker.scale.x = 0.01;
//  marker.scale.y = 0.015;
//  marker.scale.z = 0.04;

//  // Set the color -- be sure to set alpha as non-zero value!
//  // Use yellow to separate from grasp marker
//  marker.color.r = 1.0f;
//  marker.color.g = 0.0f;
//  marker.color.b = 1.0f;
//  marker.color.a = 1.0;

//  putPubMarker_.publish(marker);
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hope_node");
  
  string path_rgb;
  string path_depth;
  dataset type;
  
  if (argc == 1) {
    use_real_data_ = true;
    ROS_INFO("Using real data.");
  }
  else if (argc == 5) {
    int arg_index = 1;
    path_rgb = argv[arg_index++];
    path_depth = argv[arg_index++];
    
    roll_angle_  = atof(argv[arg_index++]);
    pitch_angle_  = atof(argv[arg_index++]);
    ROS_INFO("Using default dataset.");
    type = DEFAULT;
  }
  else if (argc == 10) {
    int arg_index = 1;
    string path_prefix = "/home/aicrobo/TUM/rgbd_dataset_freiburg1_rpy/";
    path_rgb = path_prefix + "rgb/" + argv[arg_index++];
    path_depth = path_prefix + "depth/" + argv[arg_index++];
    
    tx_ = atof(argv[arg_index++]);
    ty_ = atof(argv[arg_index++]);
    tz_ = atof(argv[arg_index++]);
    qx_ = atof(argv[arg_index++]);
    qy_ = atof(argv[arg_index++]);
    qz_ = atof(argv[arg_index++]);
    qw_ = atof(argv[arg_index++]);
    ROS_INFO("Using TUM RGB-D SLAM dataset.");
    type = TUM;
  }
  else {
    return -1;
  }
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  PlaneSegment hope(use_real_data_, base_frame_, 0.01);
  PointCloud::Ptr src_cloud(new PointCloud);
  
  if (use_real_data_) {
    while (ros::ok()) {
      // The src_cloud is actually not used here
      hope.getHorizontalPlanes(src_cloud);
    }
  }
  else {
    // Set camera pose for point cloud transferation
    hope.setParams(type, roll_angle_, pitch_angle_, tx_, ty_, tz_, qx_, qy_, qz_, qw_);
    
    // Pre-captured images are used for testing on benchmarks
    Mat rgb = imread(path_rgb);
    Mat depth = imread(path_depth, -1); // Using flag<0 to read the image without changing its type
    
    //imshow("rgb", rgb);
    //imshow("depth", depth);
    //waitKey();
    
    // The rgb images from TUM dataset are in CV_8UC3 type while the depth images are in CV_16UC1
    // The rgb image should be phrased with Vec3b while the depth with ushort
    cout << "Image type: rgb: " << rgb.type() << " depth: " << depth.type() << endl;
    
    //    // Camera intrinsic parameters for TUM RGB-D SLAM dataset
    //    float fx = 591.1; 	 	 	
    //    float fy = 590.1;
    //    float cx = 331.0;
    //    float cy = 234.0;
    
    GetCloud m_gc;
    // Filter the cloud with range 0.3-8.0m cause most RGB-D sensors are unreliable outside this range
    // But if Lidar data is used, try expand the range
    m_gc.getColorCloud(rgb, depth, src_cloud, 8.0, 0.3);
    
    hope.getHorizontalPlanes(src_cloud);
  }
  return 0;
}

/* Use example in terminal
 * hope_node 1305031128.747363.png 1305031128.754646.png 1.2788 0.5814 1.4567 0.6650 0.6515 -0.2806 -0.2334
 * 
 * rgbd_dataset_freiburg3_long_office_household/
 * 1341847980.822978.png 1341847980.822989.png -0.6821 2.6914 1.7371 0.0003 0.8609 -0.5085 -0.0151
 * 1341847996.574796.png 1341847996.574812.png 1.7985 -0.3048 1.5855 0.6819 0.5066 -0.3276 -0.4136
 * 1341848045.674943.png 1341848045.707020.png -1.5177 1.3347 1.4077 0.2563 0.8630 -0.4345 -0.0278
 * 
 * rgbd_dataset_freiburg1_rpy/
 * 1305031230.598825.png 1305031230.598843.png 1.2742 0.5998 1.5466 0.6037 0.6605 -0.4430 -0.0554

 */
