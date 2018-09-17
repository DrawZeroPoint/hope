#ifndef GET_CLOUD_H
#define GET_CLOUD_H

//STL
#include <iostream>
#include <math.h>
#include <vector>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utilities.h"

using namespace std;

class GetCloud
{
public:
  GetCloud();
  
  static bool getMonoCloud(cv::Mat depth, float fx, float fy, float cx, float cy,
                           float max_depth, float min_depth, PointCloudMono::Ptr &cloud);
  
  static bool getColorCloud(cv::Mat rgb, cv::Mat depth, float fx, float fy, float cx, float cy, 
                            float max_depth, float min_depth, PointCloud::Ptr &cloud);
  
  static bool getColorCloud(cv::Mat rgb, cv::Mat depth, PointCloud::Ptr &cloud, float max_depth, float min_depth);
  
  static bool getPoint(cv::Mat depth, int row, int col, float fx, float fy, float cx, float cy,
                       float maxDepth, float min_depth, pcl::PointXYZ &pt);
  
};

#endif // GET_CLOUD_H
