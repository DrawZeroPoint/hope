#include <iostream>

#include <Eigen/Core>

#include "hope/utilities.h"
#include "hope/pose_estimation.h"


using namespace std;
using namespace hope;


bool testQuaternionFromMatrix() {
  Eigen::Matrix4f mat;
  mat << 0, 0, 1, 0,
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 0, 1;
  Eigen::Quaternion<float> q;
  Utilities::quaternionFromMatrix(mat, q);
  cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << endl;
}

bool testCVRotatedRect() {
  Cloud_XYZ::Ptr hull(new Cloud_XYZ);
  hull->points.resize(4);
//  pcl::PointXYZ v0{1, 2, 0};
//  pcl::PointXYZ v1{1, -2, 0};
//  pcl::PointXYZ v2{-1, -2, 0};
//  pcl::PointXYZ v3{-1, 2, 0};

  // 70 deg
  pcl::PointXYZ v0{-1.5373651,  1.62373291, 0};
  pcl::PointXYZ v1{2.22140538, 0.25565233, 0};
  pcl::PointXYZ v2{1.5373651, -1.62373291, 0};
  pcl::PointXYZ v3{-2.22140538, -0.25565233, 0};

  hull->points[0] = v0;
  hull->points[1] = v1;
  hull->points[2] = v2;
  hull->points[3] = v3;

  vector<pcl::PointXY> rect;
  pcl::PointXY center{};
  pcl::PointXY edge_center{};
  float width, height, rotation;
  Utilities::getRotatedRect2D(hull, rect,center,edge_center,width,height,rotation);
}


int main(int argc, char **argv)
{
  Cloud_XYZ::Ptr contour(new Cloud_XYZ);
  contour->points.resize(4);
  contour->points[0].x = 1;
  contour->points[0].y = 1;
  contour->points[0].z = -1;
  contour->points[1].x = 1;
  contour->points[1].y = -1;
  contour->points[1].z = -1;
  contour->points[2].x = -1;
  contour->points[2].y = -1;
  contour->points[2].z = -1;
  contour->points[3].x = -1;
  contour->points[3].y = 1;
  contour->points[3].z = -1;

  pcl::PointXY p{0.9, 0};
  bool ok = Utilities::isInContour(contour, p);
  cout << "res 1: " << ok << endl;  // should be true

  p.x = 2;
  p.y = 0;
  ok = Utilities::isInContour(contour, p);
  cout << "res 2: " << ok << endl;  // should be false

  p.x = 1;
  p.y = 0;
  ok = Utilities::isInContour(contour, p);
  cout << "res 3: " << ok << endl;  // should be true

  testQuaternionFromMatrix();

  testCVRotatedRect();

//  float dsp_th = 0.005f;
//  PoseEstimation *pe = new PoseEstimation(dsp_th);
//  std::string scene_path = "/home/dzp/scene.pcd";
//  Cloud_XYZN::Ptr scene_cloud(new Cloud_XYZN);
//  pcl::io::loadPCDFile<Point_N>(scene_path, *scene_cloud);
//
//  Eigen::Matrix4f trans;
//  pe->estimate(scene_cloud, trans, true);


  return 0;
}

