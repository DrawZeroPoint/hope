#include <iostream>

#include <Eigen/Core>

#include "lib/utilities.h"
#include "lib/pose_estimation.h"


#define DEBUG

using namespace std;


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


int main(int argc, char **argv)
{
  PointCloudMono::Ptr contour(new PointCloudMono);
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

  float dsp_th = 0.005f;
  PoseEstimation *pe = new PoseEstimation(dsp_th);
  std::string scene_path = "/home/dzp/rs1.pcd";
  PointCloudN::Ptr scene_cloud(new PointCloudN);
  pcl::io::loadPCDFile<PointN>(scene_path, *scene_cloud);

  Eigen::Matrix4f trans;
  pe->estimate(scene_cloud, trans);


  return 0;
}

