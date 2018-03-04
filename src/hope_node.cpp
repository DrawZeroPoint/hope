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

// Customized message
#include "lib/get_cloud.h"
#include "lib/plane_segment.h"
#include "lib/utilities.h"

//#define DEBUG

using namespace std;
using namespace cv;

enum dataset{DEFAULT, TUM_SINGLE, TUM_LIST};

// Publishers

/// Status
bool use_real_data_ = false; // Using real-time image or imu data

/// Transform frame, only used with real time data
/// You may change the name based on your robot configuration
string base_frame_ = "base_link"; // world frame
string camera_optical_frame_ = "vision_depth_optical_frame";

/// Camera orientation params, only used for benchmarking
float roll_angle_;
float pitch_angle_;

float tx_, ty_, tz_, qx_, qy_, qz_, qw_;

void fraseInput(string input, vector<string> &vrgb, vector<string> &vdepth, 
                vector<vector<float> > &vec_xyzw)
{  
  ifstream list(input.c_str());
  
  if(!list){
    cout << "Unable to open the list" << endl;
    return;
  }
  char rgb[27];
  char depth[27];
  float s, tx, ty, tz, qx, qy, qz, qw;
  string line;

  while (getline(list, line)) {
    // http://www.cplusplus.com/reference/cstdio/scanf/
    sscanf(line.c_str(), "%f %s %s %f %f %f %f %f %f %f",
           &s, rgb, depth, &tx, &ty, &tz, &qx, &qy, &qz, &qw);
    ostringstream rgb_ost;
    rgb_ost << rgb;
    ostringstream dep_ost;
    dep_ost << depth;
    vrgb.push_back(rgb_ost.str());
    vdepth.push_back(dep_ost.str());
    vector<float> q;
    q.push_back(qx);
    q.push_back(qy);
    q.push_back(qz);
    q.push_back(qw);
    vec_xyzw.push_back(q);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hope_node");
  
  string path_prefix;
  string path_rgb;
  string path_depth;
  string path_list_all;
  dataset type;
  
  if (argc == 1) {
    use_real_data_ = true;
    ROS_INFO("Using real data.");
  }
  else if (argc == 5) {
    int arg_index = 1;
    path_rgb = argv[arg_index++]; // path to the folder of rgb images
    path_depth = argv[arg_index++]; // path to the folder of depth images
    
    roll_angle_  = atof(argv[arg_index++]); // predefined roll angle (X-axis pointing forward)
    pitch_angle_  = atof(argv[arg_index++]); // predefined pitch angle (Y-axis pointing left)
    ROS_INFO("Using default dataset.");
    type = DEFAULT;
  }
  else if (argc == 2) {
    // Test on a series of images, recommended
    int arg_index = 1;
    path_prefix = argv[arg_index++]; // path prefix of all.txt, see README for generating that
    path_list_all = path_prefix + "all.txt";
    
    ROS_INFO("Using image list of TUM RGB-D SLAM dataset.");
    type = TUM_LIST;
  }
  else if (argc == 10) {
    // Test on a single image pair
    int arg_index = 1;
    path_prefix = "/home/aicrobo/TUM/rgbd_dataset_freiburg1_desk/";
    path_rgb = path_prefix + argv[arg_index++];
    path_depth = path_prefix + argv[arg_index++];
    
    tx_ = atof(argv[arg_index++]);
    ty_ = atof(argv[arg_index++]);
    tz_ = atof(argv[arg_index++]);
    qx_ = atof(argv[arg_index++]);
    qy_ = atof(argv[arg_index++]);
    qz_ = atof(argv[arg_index++]);
    qw_ = atof(argv[arg_index++]);
    ROS_INFO("Using single pair of rgb and depth image of TUM RGB-D SLAM dataset.");
    type = TUM_SINGLE;
  }
  else {
    return -1;
  }
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  float xy_resolution = 0.03; // In meter
  float z_resolution = 0.008; // In meter
  PlaneSegment hope(use_real_data_, base_frame_, xy_resolution, z_resolution);
  PointCloud::Ptr src_cloud(new PointCloud); // Cloud input for all pipelines

  if (use_real_data_) {

    while (ros::ok()) {
      // The src_cloud is actually not used here
      hope.getHorizontalPlanes(src_cloud);
    }
  }
  else {
    if (type == DEFAULT || type == TUM_SINGLE) {
      // Set camera pose for point cloud transferation
      hope.setParams(type, roll_angle_, pitch_angle_, tx_, ty_, tz_, qx_, qy_, qz_, qw_);
      
      // Pre-captured images are used for testing on benchmarks
      Mat rgb = imread(path_rgb);
      Mat depth = imread(path_depth, -1); // Using flag<0 to read the image without changing its type
      
#ifdef DEBUG
      imshow("rgb", rgb);
      imshow("depth", depth);
      waitKey();
#endif
      
      // The rgb images from TUM dataset are in CV_8UC3 type while the depth images are in CV_16UC1
      // The rgb image should be phrased with Vec3b while the depth with ushort
      cout << "Image type: rgb: " << rgb.type() << " depth: " << depth.type() << endl;

      GetCloud m_gc;
      // Filter the cloud with range 0.3-8.0m cause most RGB-D sensors are unreliable outside this range
      // But if Lidar data is used, try expand the range
      m_gc.getColorCloud(rgb, depth, src_cloud, 8.0, 0.3);
      hope.getHorizontalPlanes(src_cloud);
    }
    else if (type == TUM_LIST) {
      vector<string> fnames_rgb;
      vector<string> fnames_depth;
      vector<vector<float> > vec_xyzw;
      fraseInput(path_list_all, fnames_rgb, fnames_depth, vec_xyzw);
      for (size_t i = 0; i < fnames_rgb.size(); ++i) {
        
        hope.setParams(type, roll_angle_, pitch_angle_, tx_, ty_, tz_,
                       vec_xyzw[i][0], vec_xyzw[i][1], vec_xyzw[i][2], vec_xyzw[i][3]);
        
        Mat rgb = imread(path_prefix + "/" + fnames_rgb[i]);
        Mat depth = imread(path_prefix + "/" + fnames_depth[i], -1);
        
#ifdef DEBUG
        imshow("rgb", rgb);
        imshow("depth", depth);
        waitKey();
#endif

        GetCloud m_gc;
        // Filter the cloud with range 0.3-8.0m cause most RGB-D sensors are unreliable outside this range
        // But if Lidar data are used, try expanding the range
        // TODO add Nan filter in this function
        m_gc.getColorCloud(rgb, depth, src_cloud, 8.0, 0.3);
        hope.getHorizontalPlanes(src_cloud);
      }
    }
  }
  return 0;
}
