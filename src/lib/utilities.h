#ifndef UTILITIES_H
#define UTILITIES_H

#include <ros/ros.h>

//STL
#include <string>
#include <vector>
#include <math.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudMono;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBN;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::PointNormal> NormalPointCloud;

class Utilities
{
public:
  Utilities();
  
  static void msgToCloud(const PointCloud::ConstPtr msg, 
                         PointCloudMono::Ptr cloud);
  
  static void estimateNorm(PointCloudMono::Ptr cloud_in, 
                           PointCloudRGBN::Ptr &cloud_out, 
                           NormalCloud::Ptr &normals_out,
                           float norm_r, float grid_sz, bool down_sp);
  
  static NormalCloud::Ptr estimateNorm(PointCloudMono::Ptr cloud_in, float norm_r);
  
  
  static void generateName(int count, string pref, string surf, string &name);
  
  static void getAverage(PointCloudMono::Ptr cloud_in, float &avr, float &deltaz);
  
  static void pointTypeTransfer(PointCloudRGBN::Ptr cloud_in, 
                                PointCloudMono::Ptr &cloud_out);
  
  static void pointTypeTransfer(PointCloud::Ptr cloud_in, 
                                PointCloudMono::Ptr &cloud_out);
  
  static void clusterExtract(PointCloudMono::Ptr cloud_in, 
                             vector<pcl::PointIndices> &cluster_indices,
                             float th_cluster, int minsize, int maxsize);
  
  static void cutCloud(pcl::ModelCoefficients::Ptr coeff_in, float th_distance,
                       PointCloudRGBN::Ptr cloud_in, vector<int> &inliers_cut,
                       PointCloudMono::Ptr &cloud_out);
  
  static void projectCloud(pcl::ModelCoefficients::Ptr coeff_in, 
                           PointCloudMono::Ptr cloud_in, 
                           PointCloudMono::Ptr &cloud_out);
  
  static void rotateCloudXY(PointCloudRGBN::Ptr cloud_in, 
                            PointCloudRGBN::Ptr &cloud_out,
                            float rx, float ry, Eigen::Matrix4f &transform_inv);
  
  static void rotateBack(PointCloudMono::Ptr cloud_in, 
                         PointCloudMono::Ptr &cloud_out,
                         Eigen::Matrix4f transform_inv);
  
  static void preProcess(PointCloudMono::Ptr cloud_in, 
                         PointCloudMono::Ptr &cloud_out, float gird_sz);
  
  static void getCloudByNorm(PointCloudRGBN::Ptr cloud_in, pcl::PointIndices::Ptr &inliers, 
                             float th_norm);
  
  static void getCloudByZ(PointCloudMono::Ptr cloud_in, pcl::PointIndices::Ptr &inliers, 
                          PointCloudMono::Ptr &cloud_out, float z_min, float z_max);
  
  static void getCloudByZ(PointCloud::Ptr cloud_in, pcl::PointIndices::Ptr &inliers, 
                          PointCloud::Ptr &cloud_out, float z_min, float z_max);
  
  static void getCloudByInliers(PointCloudMono::Ptr cloud_in, PointCloudMono::Ptr &cloud_out, 
                                pcl::PointIndices::Ptr inliers, bool negative, bool organized);
  static void getCloudByInliers(PointCloudRGBN::Ptr cloud_in, 
                                PointCloudRGBN::Ptr &cloud_out,
                                pcl::PointIndices::Ptr inliers, bool negative, bool organized);
  
  /**
   * @brief checkWithIn Check if ref_inliers contains elements in tgt_inliers
   * @param ref_inliers
   * @param tgt_inliers
   * @return rate of containment 0~1
   */
  static bool checkWithIn(pcl::PointIndices::Ptr ref_inliers, pcl::PointIndices::Ptr tgt_inliers);
  
  /**
   * @brief shrinkHull Shrink the 2D hull by distance dis according to the center
   * @param cloud Hull cloud in XY plane
   * @param cloud_sk Shrinked cloud in XY plane
   * @param dis in meter
   */
  static void shrinkHull(PointCloudMono::Ptr cloud, PointCloudMono::Ptr &cloud_sk, float dis);
  
  /**
   * @brief isInHull
   * Judge whether a given point p_in is in 2D hull, if so, return true, else return false
   * @param hull Hull cloud in XY plane
   * @param p_in Point to be judged in XY plane
   * @param offset Distance between p_in and its nearest point on hull
   * @return 
   */
  static bool isInHull(PointCloudMono::Ptr hull, pcl::PointXY p_in, 
                       pcl::PointXY &offset, pcl::PointXY &p_closest);
  
  /**
   * @brief tryExpandROI
   * Expand given ROI by pad in both xy direction
   * @param minx
   * @param miny
   * @param maxx
   * @param maxy
   * @param width Image width
   * @param height Image height
   * @param pad Expand value in px, can be negtive
   * @return false if the given ROI is abnormal, else return true
   */
  static bool tryExpandROI(int &minx, int &miny, int &maxx, int &maxy, int pad, 
                           int width = 640, int height = 480);
  
  static float pointToSegDist(float x, float y, float x1, float y1, float x2, float y2);
  
  /**
   * @brief getClosestPoint
   * Given line segment (p1,p2) and point p, get the closest point p_c of p on (p1,p2)
   * with accuracy acc
   * @param p1
   * @param p2
   * @param p
   * @param p_c
   * @param acc
   */
  static void getClosestPoint(pcl::PointXY p1, pcl::PointXY p2, 
                              pcl::PointXY p, pcl::PointXY &pc);
  
  static void smartOffset(pcl::PointXYZ &p_in, float off_xy, float off_z);
  static pcl::PolygonMesh generateMesh(const PointCloudMono::Ptr point_cloud, NormalCloud::Ptr normals);
  
  static float getCloudMeanZ(PointCloudMono::Ptr cloud_in);
  
  static float getCloudMeanZ(PointCloudRGBN::Ptr cloud_in);
  
private:
  static float determinant(float v1, float v2, float v3, float v4);
  
  static bool isIntersect(pcl::PointXY p1, pcl::PointXY p2, pcl::PointXY p3, pcl::PointXY p4);
};

#endif // UTILITIES_H
