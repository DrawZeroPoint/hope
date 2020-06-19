#ifndef UTILITIES_H
#define UTILITIES_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// STL
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/time.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/filters/filter.h>
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

#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/visualization/pcl_visualizer.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>


typedef pcl::PointNormal PointN;
typedef pcl::FPFHSignature33 FeatureFPFH;
typedef pcl::FPFHEstimationOMP<PointN, PointN, FeatureFPFH> FeatureEstimationFPFH;
typedef pcl::PointCloud<FeatureFPFH> PointCloudFPFH;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudMono;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBN;
typedef pcl::PointCloud<pcl::Normal> CloudN;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudN;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointN> ColorHandler;


class Utilities
{
public:
  Utilities();

  static bool calRANSAC(const PointCloudMono::ConstPtr& cloud_3d_in, float dt, float &grad);
  static void calRegionGrowing(PointCloudRGBN::Ptr cloud_in, int minsz, int maxsz, int nb, int smooth,
                               pcl::PointCloud<pcl::Normal>::Ptr normals, std::vector<pcl::PointIndices> &inliers);

  /**
   * @brief checkWithIn Check if ref_inliers contains elements in tgt_inliers
   * @param ref_inliers
   * @param tgt_inliers
   * @return rate of containment 0~1
   */
  static bool checkWithIn(const pcl::PointIndices::Ptr& ref_inliers, const pcl::PointIndices::Ptr& tgt_inliers);

  /**
   * Extract clusters from given point cloud with EuclideanClusterExtraction
   * @param cloud_in Mono point cloud
   * @param cluster_indices Vector of PointIndices
   * @param th_cluster ClusterTolerance
   * @param minsize MinClusterSize
   * @param maxsize MaxClusterSize
   */
  static void extractClusters(PointCloudMono::Ptr cloud_in,
                              std::vector<pcl::PointIndices> &cluster_indices,
                              float th_cluster, int minsize, int maxsize);

  template <typename T, typename U>
  static void sliceCloudWithPlane(pcl::ModelCoefficients::Ptr coeff_in, float th_distance,
                                  T cloud_in, U &cloud_out);

  static void downSampling(const PointCloudMono::Ptr& cloud_in, PointCloudMono::Ptr &cloud_out,
                           float grid_sz = 0, float z_sz = 0);

  static void downSampling(const PointCloud::Ptr& cloud_in, PointCloud::Ptr &cloud_out,
                           float grid_sz = 0, float z_sz = 0);

  static void downSampling(const PointCloudN::Ptr &cloud_in, PointCloudN::Ptr &cloud_out,
                           float grid_sz = 0, float z_sz = 0);

  static void estimateNorm(const PointCloudMono::Ptr& cloud_in,
                           PointCloudRGBN::Ptr &cloud_out,
                           CloudN::Ptr &normals_out,
                           float norm_r);

  static void estimateNorm(const PointCloudMono::Ptr& cloud_in,
                           CloudN::Ptr &normals_out,
                           float norm_r);

  static void estimateNormals(const PointCloudN::Ptr &cloud_in, PointCloudN::Ptr &cloud_out, float dsp_th);

  static void estimateFPFH(PointCloudN::Ptr cloud_in, PointCloudFPFH::Ptr &features_out, float dsp_th);

  static bool alignmentWithFPFH(PointCloudN::Ptr src_cloud, PointCloudFPFH::Ptr src_features,
                                PointCloudN::Ptr tgt_cloud, PointCloudFPFH::Ptr tgt_features,
                                Eigen::Matrix4f &transformation, PointCloudN::Ptr &src_aligned, float leaf = 0.005f);

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
  static void getClosestPoint(pcl::PointXY p1, pcl::PointXY p2, pcl::PointXY p, pcl::PointXY &pc);

  static void getCloudByInliers(const PointCloudMono::Ptr& cloud_in, PointCloudMono::Ptr &cloud_out,
                                const pcl::PointIndices::Ptr& inliers, bool negative, bool organized);

  static void getCloudByInliers(const CloudN::Ptr& cloud_in, CloudN::Ptr &cloud_out,
                                const pcl::PointIndices::Ptr& inliers, bool negative, bool organized);

  static void getCloudByInliers(const PointCloudRGBN::Ptr& cloud_in,
                                PointCloudRGBN::Ptr &cloud_out,
                                const pcl::PointIndices::Ptr& inliers, bool negative, bool organized);

  static void getCloudByNorm(const PointCloudRGBN::Ptr& cloud_in, pcl::PointIndices::Ptr &inliers,
                             float th_norm);

  static void getCloudByNorm(const CloudN::Ptr& cloud_in, pcl::PointIndices::Ptr &inliers, float th_norm);

  static void getCloudByZ(const PointCloudMono::Ptr& cloud_in, pcl::PointIndices::Ptr &inliers,
                          PointCloudMono::Ptr &cloud_out, float z_min, float z_max);

  static void getCloudByZ(const PointCloud::Ptr& cloud_in, pcl::PointIndices::Ptr &inliers,
                          PointCloud::Ptr &cloud_out, float z_min, float z_max);

  /**
   * Given a cloud, get its average, maximum, minimum, and middle z values.
   * @tparam T Cloud type, could be Mono or Colored
   * @param cloud_in
   * @param z_mean
   * @param z_max
   * @param z_min
   * @param z_mid
   */
  template <typename T>
  static void getCloudZInfo(T cloud_in, float &z_mean, float &z_max, float &z_min, float &z_mid);

  static cv::Vec3f getColorWithID(int id);

  static float getDistance(std::vector<float> v1, std::vector<float> v2);

  static pcl::PolygonMesh getMesh(PointCloudMono::Ptr point_cloud, CloudN::Ptr normals);

  static void getMinMax(std::vector<int> vec, int &minv, int &maxv);

  static std::string getName(int count, const std::string& pref, int surf);

  static void getOccupancyMap(const PointCloudMono::Ptr& cloud_src, PointCloudMono::Ptr cloud_upper, std::vector<int> occupy,
                              PointCloud::Ptr &cloud_out);

  static void getPointByZ(float z, const PointCloudMono::Ptr& cloud_in, pcl::PointXYZ &pt);

  static void heatmapRGB(float gray, uint8_t &r, uint8_t &g, uint8_t &b);

  static bool isInVector(int id, std::vector<int> vec, int &pos);
  static bool isInVector(int id, std::vector<int> &vec);

  static void matchID(std::vector<std::vector<float> > global, std::vector<std::vector<float> > local,
                      std::vector<int> in, std::vector<int> &out, int feature_dim);

  static bool mergeOrReplace(size_t g_id, std::vector<int> l_ids, size_t q_id,
                             std::vector<std::vector<float> > global, std::vector<std::vector<float> > local);

  static void msgToCloud(const PointCloud::ConstPtr& msg, PointCloudMono::Ptr cloud);

  static bool normalAnalysis(const CloudN::Ptr& cloud, float th_angle);

  static float pointToSegDist(float x, float y, float x1, float y1, float x2, float y2);

  static void convertToColorCloud(const PointCloudMono::Ptr& cloud_in,
                                  PointCloud::Ptr &cloud_out, int r, int g, int b);

  template <typename T, typename U>
  static void convertCloudType(T cloud_in, U &cloud_out);

  static void planeTo2D(float z, PointCloudMono::Ptr cloud_in,
                        PointCloudMono::Ptr &cloud_out);

  static void projectCloudTo2D(const pcl::ModelCoefficients::Ptr& coeff_in,
                               const PointCloudMono::Ptr& cloud_in,
                               PointCloudMono::Ptr &cloud_out);

  static void rotateCloudXY(const PointCloudRGBN::Ptr& cloud_in,
                            PointCloudRGBN::Ptr &cloud_out,
                            float rx, float ry, Eigen::Matrix4f &transform_inv);

  /**
   * @brief shrinkHull Shrink the 2D hull by distance dis according to the center
   * @param cloud Hull cloud in XY plane
   * @param cloud_sk Shrinked cloud in XY plane
   * @param dis in meter
   */
  static void shrinkHull(const PointCloudMono::Ptr& cloud, PointCloudMono::Ptr &cloud_sk, float dis);

  static float shortRainbowColorMap(const double &value, const double &min, const double &max);

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
  static bool tryExpandROI(int &minx, int &miny, int &maxx, int &maxy, int pad,
                           int width = 640, int height = 480);

  template <typename T>
  static void publishCloud(T cloud, const ros::Publisher& pub, std::string cloud_frame);

  template <typename T>
  static inline bool isPointCloudValid(T cloud) { return cloud->empty() == 0; }

  static bool getClustersUponPlane(const PointCloudMono::Ptr& src_cloud, const PointCloudMono::Ptr& contour, std::vector<PointCloudMono::Ptr> &clusters);

  /**
   * Determine whether a given point p in XY plane is within a contour C in the same plane.
   * The idea is, if p in C, then the angles form by (pv_i, pv_i+1) for i in [0, N] should
   * be equal to 2 PI, where N is the total number of vertexes of C. Otherwise, the
   * sum of angles should smaller than 2 PI.
   *
   * This function do require the contour is ordered clockwise or anti-clockwise.
   *
   * @param contour A point cloud of vertexes from a contour, all points should have the same z value
   * @param p a point in Z=z plane
   */
  static bool isInContour(const PointCloudMono::Ptr& contour, pcl::PointXY p);

  static void matrixToPoseArray(const Eigen::Matrix4f &mat, geometry_msgs::PoseArray &array);

  /**
   * Given a point cloud of a standing cylinder, get its pose in the scene.
   * @param cloud Cylinder cloud
   * @param pose Pose of the cylinder
   * @param z A manually given origin z value of the cylinder, if =0, calculate it with cloud,
   *          otherwise use this z for the origin. This is useful when the cylinder exceeds the
   *          sensing range of the camera.
   */
  static void getCylinderPose(const PointCloudMono::Ptr& cloud, geometry_msgs::Pose &pose, float z = 0);

  /**
   * Given a isolated box type object (isolated means that the object is not adjacent with the wall),
   * or the object cloud be
   * @param cloud Box cloud
   * @param pose Pose of the box, the origin is located at half height, center of the confronting face
   * @param z See getCylinderPose
   */
  static void getBoxPose(const PointCloudMono::Ptr& cloud, geometry_msgs::Pose &pose, float z = 0);

  /**
   * Get the bounding rect of a 2D cloud in XY plane. The rect's edges are aligned with the coordinates.
   * The rect is represented with its 4 vertexes starting from the left bottom corner anti-clockwise
   * @param cloud A 2D cloud, like a contour. If 3d cloud is given, only x and y are considered.
   * @param rect Bounding rectangle
   * @param center Center point of the rect
   * @param width Width of the rect
   * @param height Height of the rect
   */
  static void getStraightRect2D(const PointCloudMono::Ptr &cloud, std::vector<pcl::PointXY> &rect,
                                pcl::PointXY &center, float &width, float &height);

  /**
   * Given a point cloud in 2D X-Y plane, compute its hull and then using OpenCV to calculate the
   * minimum rectangle around the hull.
   * @param cloud_2d 2D point cloud
   * @param rect Rectangle vertices
   * @param center Rectangle mass center
   * @param edge_center The center of the width edge that adjacent to the observer
   * @param width Rectangle width
   * @param height Rectangle height
   * @param rotation anticlockwise rotation of the rectangle, in radius
   *
   * @attention This function has the limitation that the observer mush roughly face
   *            the box face that is used as reference (where the edge_center locates)
   * @cite https://namkeenman.wordpress.com/tag/rotatedrect/
   */
  static void getRotatedRect2D(const PointCloudMono::Ptr &cloud_2d, std::vector<pcl::PointXY> &rect,
                               pcl::PointXY &center, pcl::PointXY &edge_center, float &width, float &height, float &rotation);

  static void computeHull(PointCloudMono::Ptr cloud_2d, PointCloudMono::Ptr &cloud_hull);

  static void quaternionFromMatrix(Eigen::Matrix4f mat, Eigen::Quaternion<float> &q);

  static void quaternionFromPlanarRotation(float rotation, Eigen::Quaternion<float> &q);

private:
  static bool calNormalMean(Eigen::Matrix3Xf data, std::vector<int> part1, std::vector<int> part2,
                            Eigen::Vector3f &mean_part1, Eigen::Vector3f &mean_part2);

  static float determinant(float v1, float v2, float v3, float v4);

  static void getFurthestPointsAlongAxis(Eigen::Vector2f axis, Eigen::MatrixXf data,
                                         std::vector<int> &inlist, int &id_max, int &id_min);

  static bool isIntersect(pcl::PointXY p1, pcl::PointXY p2, pcl::PointXY p3, pcl::PointXY p4);

  static bool isInVector(int id, std::vector<std::vector<int> > vec, int &pos);

  static void matFill(std::vector<std::vector<float> > features, cv::Mat &out);

  static void matNormalize(cv::Mat query_in, cv::Mat train_in, cv::Mat &query_out, cv::Mat &train_out);

  static void searchAvailableID(std::vector<int> id_used, std::vector<int> &id_ava, size_t limit);

  static pcl::ModelCoefficients::Ptr getPlaneCoeff(float z);

  static std::vector<cv::Point2f> cloudToCVPoints(PointCloudMono::Ptr cloud_hull);

  // MeshKit
  //  https://www.mcs.anl.gov/~fathom/meshkit-docs/html/circumcenter_8cpp_source.html
  static void triCircumCenter(const float *a, float *b, float *c, pcl::PointXY &circumcenter)
  {
    float xba, yba, xca, yca;
    float balength, calength;
    float denominator;
    float xcirca, ycirca;

    /* Use coordinates relative to point `a' of the triangle. */
    xba = b[0] - a[0];
    yba = b[1] - a[1];
    xca = c[0] - a[0];
    yca = c[1] - a[1];
    /* Squares of lengths of the edges incident to `a'. */
    balength = xba * xba + yba * yba;
    calength = xca * xca + yca * yca;

    denominator = 0.5 / (xba * yca - yba * xca);

    /* Calculate offset (from `a') of circumcenter. */
    xcirca = (yca * balength - yba * calength) * denominator;
    ycirca = (xba * calength - xca * balength) * denominator;
    circumcenter.x = xcirca;
    circumcenter.y = ycirca;
  }

  static void triCircumCenter3d(double *a, double *b, double *c, double *circumcenter,
                                double *xi, double *eta)
  {
    double xba, yba, zba, xca, yca, zca;
    double balength, calength;
    double xcrossbc, ycrossbc, zcrossbc;
    double denominator;
    double xcirca, ycirca, zcirca;

    /* Use coordinates relative to point `a' of the triangle. */
    xba = b[0] - a[0];
    yba = b[1] - a[1];
    zba = b[2] - a[2];
    xca = c[0] - a[0];
    yca = c[1] - a[1];
    zca = c[2] - a[2];
    /* Squares of lengths of the edges incident to `a'. */
    balength = xba * xba + yba * yba + zba * zba;
    calength = xca * xca + yca * yca + zca * zca;

    /* Take your chances with floating-point roundoff. */
    xcrossbc = yba * zca - yca * zba;
    ycrossbc = zba * xca - zca * xba;
    zcrossbc = xba * yca - xca * yba;

    /* Calculate the denominator of the formulae. */
    denominator = 0.5 / (xcrossbc * xcrossbc + ycrossbc * ycrossbc +
                         zcrossbc * zcrossbc);

    /* Calculate offset (from `a') of circumcenter. */
    xcirca = ((balength * yca - calength * yba) * zcrossbc -
              (balength * zca - calength * zba) * ycrossbc) * denominator;
    ycirca = ((balength * zca - calength * zba) * xcrossbc -
              (balength * xca - calength * xba) * zcrossbc) * denominator;
    zcirca = ((balength * xca - calength * xba) * ycrossbc -
              (balength * yca - calength * yba) * xcrossbc) * denominator;
    circumcenter[0] = xcirca;
    circumcenter[1] = ycirca;
    circumcenter[2] = zcirca;

    if (xi != (double *) NULL) {
      /* To interpolate a linear function at the circumcenter, define a     */
      /*   coordinate system with a xi-axis directed from `a' to `b' and    */
      /*   an eta-axis directed from `a' to `c'.  The values for xi and eta */
      /*   are computed by Cramer's Rule for solving systems of linear      */
      /*   equations.                                                       */

      /* There are three ways to do this calculation - using xcrossbc, */
      /*   ycrossbc, or zcrossbc.  Choose whichever has the largest    */
      /*   magnitude, to improve stability and avoid division by zero. */
      if (((xcrossbc >= ycrossbc) ^ (-xcrossbc > ycrossbc)) &&
          ((xcrossbc >= zcrossbc) ^ (-xcrossbc > zcrossbc))) {
        *xi = (ycirca * zca - zcirca * yca) / xcrossbc;
        *eta = (zcirca * yba - ycirca * zba) / xcrossbc;
      } else if ((ycrossbc >= zcrossbc) ^ (-ycrossbc > zcrossbc)) {
        *xi = (zcirca * xca - xcirca * zca) / ycrossbc;
        *eta = (xcirca * zba - zcirca * xba) / ycrossbc;
      } else {
        *xi = (xcirca * yca - ycirca * xca) / zcrossbc;
        *eta = (ycirca * xba - xcirca * yba) / zcrossbc;
      }
    }
  }

  static void triCircumCenter2D(float *a, float *b, float *c, pcl::PointXY &result)
  {
    triCircumCenter(a, b, c, result);

    result.x += a[0];
    result.y += a[1];
  }

  static void triCircumCenter3D(double *a, double *b, double *c, double *result)
  {
    double xi, eta;
    triCircumCenter3d(a, b, c, result, &xi, &eta);
    result[0] += a[0];
    result[1] += a[1];
    result[2] += a[2];
  }
};

#endif // UTILITIES_H
