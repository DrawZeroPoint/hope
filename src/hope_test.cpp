#include <iostream>

#include "lib/utilities.h"

#define DEBUG

using namespace std;


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

  pcl::PointXY p;
  p.x = 0.9;
  p.y = 0;
  bool ok = Utilities::isInContour(contour, p);
  cerr << "res 1: " << ok << endl;  // should be true

  p.x = 2;
  p.y = 0;
  ok = Utilities::isInContour(contour, p);
  cerr << "res 2: " << ok << endl;  // should be false

  p.x = 1;
  p.y = 0;
  ok = Utilities::isInContour(contour, p);
  cerr << "res 3: " << ok << endl;  // should be false

  return 0;
}

