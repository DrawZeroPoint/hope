/* This cpp is for producing the experiment result in the paper titled:
   HoPE: Horizontal Plane Extractor for Cluttered 3D Scenes
   For a handy tool for extracting horizontal planes in ROS, please use hope_ros
*/

#include <geometry_msgs/TransformStamped.h>

#include <math.h>
#include <string.h>
#include <iostream> 
#include <iomanip>
#include <fstream>  

#include <cv_bridge/cv_bridge.h>

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

// Transform frame, only used with real time data
// You may change the name based on your robot configuration
string base_frame_ = "mobile_base_link"; // world frame
string camera_optical_frame_ = "/oil/perception/head_camera/cloud";

/// Camera orientation params, only used for benchmarking
float roll_angle_ = 0.0;
float pitch_angle_ = 0.0;
float yaw_angle_ = 0.0;

float tx_, ty_, tz_, qx_, qy_, qz_, qw_;

void printHelp()
{
  cout << "hope_node VARIABLES" << endl <<
       "VARIABLES number could be 1, 2, 4, or 10:" << endl <<
       "1 VARIABLE, test a sequence on TUM dataset, {TUM_DATASET_FOLDER}" << endl <<
       "Example: ./hope_node /path/to/TUM/rgbd_dataset_freiburg1_desk" << endl << endl <<
       "2 VARIABLES, test on point cloud file, {POINT_CLOUD_FILE} {TYPE}, TYPE could be ply or pcd" << endl <<
       "Example: ./hope_node ~/loft.ply ply" << endl << endl <<
       "3 VARIABLES, test with the synthetic object, set the pose of the object with {ROLL} {PITCH} {YAW}" << endl <<
       "Example: ./hope_node 0 0 0" << endl << endl <<
       "10 VARIABLES, test on single TUM pair, {TUM_DATASET_FOLDER} {RGB_IMG} {DEPTH_IMG} {TX} {TY} {TZ} {QX} {QY} {QZ} {QW}" << endl <<
       "Example: ./hope_node ~/TUM/rgbd_dataset_freiburg1_desk/ rgb/1305031459.259760.png depth/1305031459.274941.png "
       "-0.2171 -0.0799 1.3959 -0.8445 -0.0451 0.0954 0.5251";
}

void phaseInput(string input, vector<string> &vrgb, vector<string> &vdepth,
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
    tx_ = tx;
    ty_ = ty;
    tz_ = tz;
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
  // You need to run roscore to launch this program
  ros::init(argc, argv, "hope_node");
  
  // TUM
  

  return 0;
}
