#include "get_cloud.h"

using namespace cv;

GetCloud::GetCloud()
{
}

template<class T>
inline bool uIsFinite(const T & value)
{
#if _MSC_VER
  return _finite(value) != 0;
#else
  return std::isfinite(value);
#endif
}

float getDepth(const Mat &depthImage, int x, int y,
               bool smoothing, float maxZError, 
               bool estWithNeighborsIfNull)
{
  int u = x;
  int v = y;
  
  bool isInMM = depthImage.type() == CV_16UC1; // is in mm?
  
  // Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
  // https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
  // Window weights:
  //  | 1 | 2 | 1 |
  //  | 2 | 4 | 2 |
  //  | 1 | 2 | 1 |
  int u_start = max(u - 1, 0);
  int v_start = max(v - 1, 0);
  int u_end = min(u + 1, depthImage.cols - 1);
  int v_end = min(v + 1, depthImage.rows - 1);
  
  float depth = 0.0f;
  if(isInMM) {
    if(depthImage.at<unsigned short>(v, u) > 0 &&
       depthImage.at<unsigned short>(v, u) < numeric_limits<unsigned short>::max()) {
      depth = float(depthImage.at<unsigned short>(v, u)) * 0.001f;
    }
  }
  else
    depth = depthImage.at<float>(v, u);
  
  if((depth == 0.0f || !uIsFinite(depth)) && estWithNeighborsIfNull) {
    // all cells no2 must be under the zError to be accepted
    float tmp = 0.0f;
    int count = 0;
    for(int uu = u_start; uu <= u_end; ++uu) {
      for(int vv = v_start; vv <= v_end; ++vv) {
        if((uu == u && vv != v) || (uu != u && vv == v)) {
          float d = 0.0f;
          if(isInMM) {
            if(depthImage.at<unsigned short>(vv, uu) > 0 &&
               depthImage.at<unsigned short>(vv, uu) < numeric_limits<unsigned short>::max()) {
              depth = float(depthImage.at<unsigned short>(vv, uu)) * 0.001f;
            }
          }
          else {
            d = depthImage.at<float>(vv, uu);
          }
          if(d != 0.0f && uIsFinite(d)) {
            if(tmp == 0.0f) {
              tmp = d;
              ++count;
            }
            else if(fabs(d - tmp) < maxZError)
            {
              tmp += d;
              ++count;
            }
          }
        }
      }
    }
    if(count > 1) {
      depth = tmp / float(count);
    }
  }
  
  if(depth != 0.0f && uIsFinite(depth)) {
    if(smoothing) {
      float sumWeights = 0.0f;
      float sumDepths = 0.0f;
      for(int uu = u_start; uu <= u_end; ++uu) {
        for(int vv = v_start; vv <= v_end; ++vv) {
          if(!(uu == u && vv == v)) {
            float d = 0.0f;
            if(isInMM) {
              if(depthImage.at<unsigned short>(vv,uu) > 0 &&
                 depthImage.at<unsigned short>(vv,uu) < numeric_limits<unsigned short>::max()) {
                depth = float(depthImage.at<unsigned short>(vv,uu))*0.001f;
              }
            }
            else {
              d = depthImage.at<float>(vv,uu);
            }
            
            // ignore if not valid or depth difference is too high
            if(d != 0.0f && uIsFinite(d) && fabs(d - depth) < maxZError)
            {
              if(uu == u || vv == v)
              {
                sumWeights += 2.0f;
                d *= 2.0f;
              }
              else {
                sumWeights+=1.0f;
              }
              sumDepths += d;
            }
          }
        }
      }
      // set window weight to center point
      depth *= 4.0f;
      sumWeights += 4.0f;
      
      // mean
      depth = (depth+sumDepths)/sumWeights;
    }
  }
  else {
    depth = 0;
  }
  return depth;
}


bool projectDepthTo3D(const Mat &depthImage, pcl::PointXYZ &pt,
                      float max_depth, float min_depth,
                      int x, int y,
                      float cx, float cy, float fx, float fy, 
                      bool smoothing, float maxZError)
{
  float depth = getDepth(depthImage, x, y, smoothing, maxZError, true);
  if(depth > 0.0f && depth < max_depth && depth > min_depth) {
    // Use correct principal point from calibration
    cx = cx > 0.0f ? cx : float(depthImage.cols/2) - 0.5f; //cameraInfo.K.at(2)
    cy = cy > 0.0f ? cy : float(depthImage.rows/2) - 0.5f; //cameraInfo.K.at(5)
    
    // Fill in XYZ
    pt.x = (x - cx) * depth / fx;
    pt.y = (y - cy) * depth / fy;
    pt.z = depth;
    return true;
  }
  else {
    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
    return false;
  }
}

bool GetCloud::getMonoCloud(Mat depth, float fx, float fy, float cx, float cy, 
                            float max_depth, float min_depth, PointCloudMono::Ptr &cloud)
{
  cloud->height = depth.rows;
  cloud->width  = depth.cols;
  cloud->is_dense = false;
  cloud->resize(cloud->height * cloud->width);
  
  size_t i = 0;
  for (PointCloudMono::iterator pit = cloud->begin(); 
       pit != cloud->end(); ++pit) {
    pcl::PointXYZ pt;
    projectDepthTo3D(depth, pt, max_depth, min_depth, 
                     i%640, i/640, cx, cy, fx, fy, false, 3.0);
    pit->x = pt.x;
    pit->y = pt.y;
    pit->z = pt.z;
    ++i;
  }
}

bool GetCloud::getColorCloud(Mat rgb, Mat depth, float fx, float fy, float cx, float cy, 
                             float max_depth, float min_depth, PointCloud::Ptr &cloud)
{
  cloud->height = depth.rows;
  cloud->width  = depth.cols;
  cloud->is_dense = false;
  cloud->resize(cloud->height * cloud->width);
  
  size_t i = 0;
  for (PointCloud::iterator pit = cloud->begin(); 
       pit != cloud->end(); ++pit) {
    pcl::PointXYZ pt;
    projectDepthTo3D(depth, pt, max_depth, min_depth, 
                     i%640, i/640, cx, cy, fx, fy, false, 3.0);
    
    Vec3b c = rgb.at<Vec3b>(i%640, i/640);
    
    pit->x = pt.x;
    pit->y = pt.y;
    pit->z = pt.z;
    pit->r = c[0];
    pit->g = c[1];
    pit->b = c[2];
    ++i;
  }
}

bool GetCloud::getColorCloud(Mat rgb, Mat depth, PointCloud::Ptr &cloud,
                             float max_depth, float min_depth)
{
  cloud->height = depth.rows;
  cloud->width  = depth.cols;
  cloud->is_dense = false;
  cloud->resize(cloud->height * cloud->width);
  // Param for TMU dataset
  float focalLength = 517.0;
  float centerX = 318.6;
  float centerY = 255.3;
  float scalingFactor = 5000.0;
  
  size_t i = 0;
  for (PointCloud::iterator pit = cloud->begin(); 
       pit != cloud->end(); ++pit) {
    float x, y;
    int c = i % 640;
    int r = i / 640;
    float z = float(depth.at<ushort>(r, c)) / scalingFactor;
    if(z < max_depth && z > min_depth) {
      x = (c - centerX) * z / focalLength;
      y = (r - centerY) * z / focalLength;
    }
    else {
      x = y = z = std::numeric_limits<float>::quiet_NaN();
    }
    
    Vec3b vc = rgb.at<Vec3b>(r, c);
    
    pit->x = x;
    pit->y = y;
    pit->z = z;
    pit->r = vc[2];
    pit->g = vc[1];
    pit->b = vc[0];
    ++i;
  }

}

bool GetCloud::getPoint(cv::Mat depth, int row, int col, 
                        float fx, float fy, float cx, float cy,
                        float max_depth, float min_depth, pcl::PointXYZ &pt)
{
  bool success = projectDepthTo3D(depth, pt, max_depth, min_depth,
                                  col, row, cx, cy, fx, fy, false, 3.0);
  return success;
}
