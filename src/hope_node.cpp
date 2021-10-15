/* This cpp is for producing the experiment result in the paper titled:
   HoPE: Horizontal Plane Extractor for Cluttered 3D Scenes
   For a handy tool for extracting horizontal planes in CLOUD_STREAM, please use hope_ros
*/

#include <cmath>
#include <cstring>
#include <iostream>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// HoPE
#include "hope/get_cloud.h"
#include "hope/plane_segment.h"
#include "hope/utilities.h"
#include "hope/transform.h"


using namespace std;
using namespace hope;

float tx_, ty_, tz_, qx_, qy_, qz_, qw_;


void printHelp()
{
  cout << "hope_node VARIABLES" << endl <<
       "VARIABLES number could be 1, 2, 4, or 10:" << endl <<
       "1 VARIABLE, test a sequence on TUM dataset, {TUM_DATASET_FOLDER}" << endl <<
       "Example: ./hope_node /path/to/TUM/rgbd_dataset_freiburg1_desk" << endl << endl <<
       "2 VARIABLES, test on point cloud file, {POINT_CLOUD_FILE} {TYPE}, TYPE could be ply or pcd" << endl <<
       "Example: ./hope_node ~/loft.ply ply" << endl << endl <<
       "10 VARIABLES, test on single TUM pair, {TUM_DATASET_FOLDER} {RGB_IMG} {DEPTH_IMG} {TX} {TY} {TZ} {QX} {QY} {QZ} {QW}" << endl <<
       "Example: ./hope_node ~/TUM/rgbd_dataset_freiburg1_desk/ rgb/1305031459.259760.png depth/1305031459.274941.png "
       "-0.2171 -0.0799 1.3959 -0.8445 -0.0451 0.0954 0.5251";
}

void phaseInput(const string& input, vector<string> &vrgb, vector<string> &vdepth,
                vector<vector<float> > &vec_xyzw)
{
  ifstream list(input.c_str());

  if (!list) {
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
    type = TUM_LIST;
    path_prefix = argv[2]; // path prefix of all.txt, see README for the method to generate it
    path_list_all = path_prefix + "/all.txt";
    cout << "Using the image list from TUM RGB-D SLAM dataset." << endl;
  } else if (argc == 3) {
    type = PCD_PLY;
    path_cloud = argv[2];
    cloud_type = argv[3];
    cout << "Using point cloud file " << path_cloud << endl;
  } else if (argc == 11) {
    type = TUM_SINGLE;
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
    cout << "Using a single RGB-D pair from the TUM RGB-D SLAM dataset." << endl;
  } else {
    printHelp();
    return EXIT_FAILURE;
  }

  // float xy_resolution = 0.02; // In meter
  // float z_resolution = 0.004; // In meter
  float xy_resolution = 0.05; // In meter
  float z_resolution = 0.02; // In meter
  cout << "Using threshold: xy@" << xy_resolution << " " << "z@" << z_resolution << endl;

  hope::PlaneSegment hope(xy_resolution, z_resolution, type, true);

  // Cloud input for all pipelines
  Cloud_XYZRGB::Ptr src_cloud(new Cloud_XYZRGB);
  Cloud_XYZRGB::Ptr t_cloud(new Cloud_XYZRGB);

  cv::Mat rgb;
  cv::Mat depth;

  switch (type) {
    case PCD_PLY: {
      if (cloud_type == "pcd") {
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_cloud, *src_cloud) == -1) {
          cerr << "Couldn't read pcd file." << endl;
          return EXIT_FAILURE;
        }
      } else if (cloud_type == "ply") {
        pcl::PLYReader Reader;
        Reader.read(path_cloud, *src_cloud);
      } else {
        cerr << "Unrecognized file format." << endl;
        return EXIT_FAILURE;
      }
      hope::Transform::doTransform(src_cloud, t_cloud, qx_, qy_, qz_, qw_);
      hope.getHorizontalPlanes(t_cloud);
    }
    case TUM_SINGLE: {
      // Pre-captured images are used for testing on benchmarks
      rgb = cv::imread(path_rgb);
      depth = cv::imread(path_depth, -1); // Using flag<0 to read the image without changing its type

      // The rgb images from TUM dataset are in CV_8UC3 type while the depth images are in CV_16UC1
      // The rgb image should be phrased with Vec3b while the depth with ushort
      cout << "Image type: rgb: " << rgb.type() << " depth: " << depth.type() << endl;
      cv::imshow("rgb", rgb);
      cv::imshow("depth", depth);
      cv::waitKey();

      // Filter the cloud with range 0.3-8.0m cause most RGB-D sensors are unreliable outside this range
      // But if Lidar data are used, try expanding the range
      // TODO add Nan filter in this function
      GetCloud::getColorCloud(rgb, depth, src_cloud, 8.0, 0.3);
      hope::Transform::doTransform(src_cloud, t_cloud, qx_, qy_, qz_, qw_, tx_, ty_, tz_);
      hope.getHorizontalPlanes(t_cloud);
    }
    case TUM_LIST: {
      vector<string> rgb_names;
      vector<string> depth_names;
      vector<vector<float> > vec_quat;
      phaseInput(path_list_all, rgb_names, depth_names, vec_quat);

      for (size_t i = 0; i < rgb_names.size(); ++i) {
        cout << "Processing: " << rgb_names[i] << endl;
        rgb = cv::imread(path_prefix + "/" + rgb_names[i]);
        depth = cv::imread(path_prefix + "/" + depth_names[i], -1);
        GetCloud::getColorCloud(rgb, depth, src_cloud, 8.0, 0.3);
        hope::Transform::doTransform(
            src_cloud, t_cloud,
            vec_quat[i][0], vec_quat[i][1], vec_quat[i][2], vec_quat[i][3], tx_, ty_, tz_
        );
        hope.getHorizontalPlanes(t_cloud);
      }
    }
    case CLOUD_STREAM:
      break;
  }

  return EXIT_SUCCESS;
}
