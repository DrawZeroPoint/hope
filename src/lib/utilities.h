#ifndef UTILITIES_H
#define UTILITIES_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// STL
#include <iostream> 
#include <string>
#include <vector>
#include <math.h>

// PCL
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

#include <pcl_conversions/pcl_conversions.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudMono;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBN;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::PointNormal> NormalPointCloud;

class Utilities
{
public:
  Utilities();

  bool calRANSAC(const PointCloudMono::ConstPtr cloud_3d_in, float dt, float &grad);
  void calRegionGrowing(PointCloudRGBN::Ptr cloud_in, int minsz, int maxsz, int nb, int smooth,
                        pcl::PointCloud<pcl::Normal>::Ptr normals, std::vector<pcl::PointIndices> &inliers);
  
  /**
   * @brief checkWithIn Check if ref_inliers contains elements in tgt_inliers
   * @param ref_inliers
   * @param tgt_inliers
   * @return rate of containment 0~1
   */
  bool checkWithIn(pcl::PointIndices::Ptr ref_inliers, pcl::PointIndices::Ptr tgt_inliers);
  
  void clusterExtract(PointCloudMono::Ptr cloud_in,
                      std::vector<pcl::PointIndices> &cluster_indices,
                      float th_cluster, int minsize, int maxsize);
  
  void cutCloud(pcl::ModelCoefficients::Ptr coeff_in, float th_distance,
                PointCloudRGBN::Ptr cloud_in, std::vector<int> &inliers_cut,
                PointCloudMono::Ptr &cloud_out);
  
  void cutCloud(pcl::ModelCoefficients::Ptr coeff_in, float th_distance,
                PointCloudMono::Ptr cloud_in, std::vector<int> &inliers_cut,
                PointCloudMono::Ptr &cloud_out);
  
  void downSampling(PointCloudMono::Ptr cloud_in,
                    PointCloudMono::Ptr &cloud_out, float gird_sz, float z_sz );

  void downSampling(PointCloud::Ptr cloud_in, PointCloud::Ptr &cloud_out,
                    float gird_sz, float z_sz);
  
  void estimateNorm(PointCloudMono::Ptr cloud_in,
                    PointCloudRGBN::Ptr &cloud_out,
                    NormalCloud::Ptr &normals_out,
                    float norm_r);
  
  void estimateNorm(PointCloudMono::Ptr cloud_in,
                    NormalCloud::Ptr &normals_out,
                    float norm_r);
  
  void getAverage(PointCloudMono::Ptr cloud_in, float &avr, float &deltaz);
  
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
  void getClosestPoint(pcl::PointXY p1, pcl::PointXY p2,
                       pcl::PointXY p, pcl::PointXY &pc);
  
  void getCloudByInliers(PointCloudMono::Ptr cloud_in, PointCloudMono::Ptr &cloud_out,
                         pcl::PointIndices::Ptr inliers, bool negative, bool organized);
  
  void getCloudByInliers(NormalCloud::Ptr cloud_in, NormalCloud::Ptr &cloud_out,
                         pcl::PointIndices::Ptr inliers, bool negative, bool organized);
  
  void getCloudByInliers(PointCloudRGBN::Ptr cloud_in,
                         PointCloudRGBN::Ptr &cloud_out,
                         pcl::PointIndices::Ptr inliers, bool negative, bool organized);
  
  void getCloudByNorm(PointCloudRGBN::Ptr cloud_in, pcl::PointIndices::Ptr &inliers,
                      float th_norm);
  
  void getCloudByNorm(NormalCloud::Ptr cloud_in, pcl::PointIndices::Ptr &inliers, float th_norm);
  
  void getCloudByZ(PointCloudMono::Ptr cloud_in, pcl::PointIndices::Ptr &inliers,
                   PointCloudMono::Ptr &cloud_out, float z_min, float z_max);
  
  void getCloudByZ(PointCloud::Ptr cloud_in, pcl::PointIndices::Ptr &inliers,
                   PointCloud::Ptr &cloud_out, float z_min, float z_max);
  
  float getCloudMeanZ(PointCloudMono::Ptr cloud_in);
  
  float getCloudMeanZ(PointCloudRGBN::Ptr cloud_in);
  
  cv::Vec3f getColorWithID(int id);
  
  float getDistance(std::vector<float> v1, std::vector<float> v2);

  void getHullCenter(PointCloud::Ptr hull, float &x, float &y);
  
  pcl::PolygonMesh getMesh(const PointCloudMono::Ptr point_cloud, NormalCloud::Ptr normals);
  
  void getMinMax(std::vector<int> vec, int &minv, int &maxv);
  
  std::string getName(int count, std::string pref, int surf);
  
  void getOccupancyMap(PointCloudMono::Ptr cloud_src, PointCloudMono::Ptr cloud_upper, std::vector<int> occupy,
                       PointCloud::Ptr &cloud_out);

  void getPointByZ(float z, PointCloudMono::Ptr cloud_in, pcl::PointXYZ &pt);

  static void heatmapRGB(float gray, uint8_t &r, uint8_t &g, uint8_t &b);
  
  /**
   * @brief isInHull
   * Judge whether a given point p_in is in 2D hull, if so, return true, else return false
   * @param hull Hull cloud in XY plane
   * @param p_in Point to be judged in XY plane
   * @param offset Distance between p_in and its nearest point on hull
   * @return
   */
  bool isInHull(PointCloudMono::Ptr hull, pcl::PointXY p_in,
                pcl::PointXY &offset, pcl::PointXY &p_closest);
  
  bool isInVector(int id, std::vector<int> vec, int &pos);
  bool isInVector(int id, std::vector<int> &vec);

  void matchID(std::vector<std::vector<float> > global, std::vector<std::vector<float> > local, std::vector<int> in,
               std::vector<int> &out, int feature_dim);

  bool mergeOrReplace(size_t g_id, std::vector<int> l_ids, size_t q_id,
                      std::vector<std::vector<float> > global, std::vector<std::vector<float> > local);
  
  void msgToCloud(const PointCloud::ConstPtr msg,
                  PointCloudMono::Ptr cloud);

  bool normalAnalysis(NormalCloud::Ptr cloud, float th_angle);

  bool pcaAnalysis(pcl::PointXYZ pointMaxZ, pcl::PointXYZ pointMinZ,
                   const PointCloudMono::ConstPtr cloud_3d_in, float &proj, float &grad);
  
  float pointToSegDist(float x, float y, float x1, float y1, float x2, float y2);
  
  void pointTypeTransfer(PointCloudRGBN::Ptr cloud_in,
                         PointCloudMono::Ptr &cloud_out);
  
  void pointTypeTransfer(PointCloud::Ptr cloud_in,
                         PointCloudMono::Ptr &cloud_out);
  
  void pointTypeTransfer(PointCloudMono::Ptr cloud_in,
                         PointCloud::Ptr &cloud_out, int r, int g, int b);
  
  void planeTo2D(float z, PointCloudMono::Ptr cloud_in,
                 PointCloudMono::Ptr &cloud_out);
  
  void projectCloud(pcl::ModelCoefficients::Ptr coeff_in,
                    PointCloudMono::Ptr cloud_in,
                    PointCloudMono::Ptr &cloud_out);
  
  void rotateCloudXY(PointCloudRGBN::Ptr cloud_in,
                     PointCloudRGBN::Ptr &cloud_out,
                     float rx, float ry, Eigen::Matrix4f &transform_inv);
  
  void rotateBack(PointCloudMono::Ptr cloud_in,
                  PointCloudMono::Ptr &cloud_out,
                  Eigen::Matrix4f transform_inv);
  
  /**
   * @brief shrinkHull Shrink the 2D hull by distance dis according to the center
   * @param cloud Hull cloud in XY plane
   * @param cloud_sk Shrinked cloud in XY plane
   * @param dis in meter
   */
  void shrinkHull(PointCloudMono::Ptr cloud, PointCloudMono::Ptr &cloud_sk, float dis);
  
  float shortRainbowColorMap(const double &value, const double &min, const double &max);
  /**
   * @brief tryExpandROI
   * Expand given ROI by pad in both xy direction
   * @param minx
   * @param miny
   * @param maxx
   * @param maxy
   * @param width Image width
   * @param height Image height
   * @param pad Expand value in px, can be negative
   * @return false if the given ROI is abnormal, else return true
   */
  bool tryExpandROI(int &minx, int &miny, int &maxx, int &maxy, int pad,
                    int width = 640, int height = 480);

  static void publishCloud(PointCloud::Ptr cloud, const ros::Publisher& pub, std::string cloud_frame);
  static void publishCloud(PointCloudMono::Ptr cloud, const ros::Publisher& pub, std::string cloud_frame);

private:
  bool calNormalMean(Eigen::Matrix3Xf data, std::vector<int> part1, std::vector<int> part2,
                     Eigen::Vector3f &mean_part1, Eigen::Vector3f &mean_part2);

  float determinant(float v1, float v2, float v3, float v4);

  void getFurthestPointsAlongAxis(Eigen::Vector2f axis, Eigen::MatrixXf data,
                                  std::vector<int> &inlist, int &id_max, int &id_min);

  bool isIntersect(pcl::PointXY p1, pcl::PointXY p2, pcl::PointXY p3, pcl::PointXY p4);

  bool isInVector(int id, std::vector<std::vector<int> > vec, int &pos);

  void matFill(std::vector<std::vector<float> > features, cv::Mat &out);

  void matNormalize(cv::Mat query_in, cv::Mat train_in, cv::Mat &query_out, cv::Mat &train_out);

  void searchAvailableID(std::vector<int> id_used, std::vector<int> &id_ava, size_t limit);
};

#endif // UTILITIES_H
