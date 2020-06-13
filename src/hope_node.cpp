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
  string path_prefix;
  string path_rgb;
  string path_depth;
  string path_list_all;
  data_type type;

  // Indoor RGB-D && synthesized
  string path_cloud;
  string cloud_type;

  if (argc == 2) {
    // Test on a series of images, recommended
    // Example: ./hope_node ~/TUM/rgbd_dataset_freiburg1_desk
    path_prefix = argv[2]; // path prefix of all.txt, see README for the method to generate that
    path_list_all = path_prefix + "/all.txt";

    cout << "Using the image list from TUM RGB-D SLAM dataset." << endl;
    type = TUM_LIST;
  }
  else if (argc == 3) {
    // Test on a PCD or PLY point cloud
    // Example: ./hope_node ~/loft.ply ply
    path_cloud = argv[2];
    cloud_type = argv[3];
    cout << "Using point cloud file " << path_cloud << endl;
    type = POINT_CLOUD;
  }
  else if (argc == 4) {
    type = SYN;
    roll_angle_ = atof(argv[2]);
    pitch_angle_ = atof(argv[3]);
    yaw_angle_ = atof(argv[4]);
    cout << "Using synthetic object." << endl;
  }
  else if (argc == 11) {
    // Test on a single image pair
    /*
    Example: ./hope_node ~/TUM/rgbd_dataset_freiburg1_desk/ rgb/1305031459.259760.png depth/1305031459.274941.png \
    -0.2171 -0.0799 1.3959 -0.8445 -0.0451 0.0954 0.5251
    */
    int arg_index = 1;
    path_prefix = argv[arg_index++];
    path_rgb = path_prefix + argv[arg_index++];
    path_depth = path_prefix + argv[arg_index++];
    
    tx_ = atof(argv[arg_index++]);
    ty_ = atof(argv[arg_index++]);
    tz_ = atof(argv[arg_index++]);
    qx_ = atof(argv[arg_index++]);
    qy_ = atof(argv[arg_index++]);
    qz_ = atof(argv[arg_index++]);
    qw_ = atof(argv[arg_index++]);
    cout << "Using single pair of rgb and depth image of TUM RGB-D SLAM dataset." << endl;
    type = TUM_SINGLE;
  }
  else {
    printHelp();
    return -1;
  }

  // float xy_resolution = 0.02; // In meter
  // float z_resolution = 0.004; // In meter
  float xy_resolution = 0.05; // In meter
  float z_resolution = 0.02; // In meter
  cout << "Using threshold: xy@" << xy_resolution << " " << "z@" << z_resolution << endl;

  PlaneSegment hope(xy_resolution, z_resolution);
  PointCloud::Ptr src_cloud(new PointCloud); // Cloud input for all pipelines

  hope.setMode(type);
  if (type == SYN) {
    // -2.0944 0 0
    hope.setRPY(roll_angle_, pitch_angle_, yaw_angle_);
    while (true) {
      hope.getHorizontalPlanes(src_cloud);
    }
  }
  else {
    if (type == POINT_CLOUD) {
      if (cloud_type == "pcd") {
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_cloud, *src_cloud) == -1) {
          cerr << "Couldn't read pcd file." << endl;
          return -1;
        }
      }
      else if (cloud_type == "ply") {
        pcl::PLYReader Reader;
        Reader.read(path_cloud, *src_cloud);
      }
      else {
        cerr << "Unrecognized file format." << endl;
        return -1;
      }
      hope.setQ(qx_, qy_, qz_, qw_);
      hope.getHorizontalPlanes(src_cloud);
    }

    cv::Mat rgb;
    cv::Mat depth;
    GetCloud m_gc;

    if (type == TUM_SINGLE) {
      // Set camera pose for point cloud transformation
      hope.setQ(qx_, qy_, qz_, qw_);
      hope.setT(tx_, ty_, tz_);
      
      // Pre-captured images are used for testing on benchmarks
      rgb = cv::imread(path_rgb);
      depth = cv::imread(path_depth, -1); // Using flag<0 to read the image without changing its type

#ifdef DEBUG
      // The rgb images from TUM dataset are in CV_8UC3 type while the depth images are in CV_16UC1
      // The rgb image should be phrased with Vec3b while the depth with ushort
      cout << "Image type: rgb: " << rgb.type() << " depth: " << depth.type() << endl;
      cv::imshow("rgb", rgb);
      cv::imshow("depth", depth);
      cv::waitKey();
#endif

      // Filter the cloud with range 0.3-8.0m cause most RGB-D sensors are unreliable outside this range
      // But if Lidar data are used, try expanding the range
      // TODO add Nan filter in this function
      GetCloud::getColorCloud(rgb, depth, src_cloud, 8.0, 0.3);
      assert(Utilities::isPointCloudValid(src_cloud));
      hope.getHorizontalPlanes(src_cloud);
    }
    else if (type == TUM_LIST) {
      vector<string> fnames_rgb;
      vector<string> fnames_depth;
      vector<vector<float> > vec_xyzw;
      phaseInput(path_list_all, fnames_rgb, fnames_depth, vec_xyzw);
      for (size_t i = 0; i < fnames_rgb.size(); ++i) {
        
        hope.setQ(vec_xyzw[i][0], vec_xyzw[i][1], vec_xyzw[i][2], vec_xyzw[i][3]);
        hope.setT(tx_, ty_, tz_);
        
        cout << "Processing: " << fnames_rgb[i] << endl;
        rgb = cv::imread(path_prefix + "/" + fnames_rgb[i]);
        depth = cv::imread(path_prefix + "/" + fnames_depth[i], -1);

        // Filter the cloud with range 0.3-8.0m cause most RGB-D sensors are unreliable outside this range
        // But if Lidar data are used, try expanding the range
        m_gc.getColorCloud(rgb, depth, src_cloud, 8.0, 0.3);
        hope.getHorizontalPlanes(src_cloud);
      }
    }
  }

  return 0;
}
