#ifndef PLANE_SEGMENT_H
#define PLANE_SEGMENT_H

// ROS
#include <ros/ros.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/surface/poisson.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>

#include <boost/thread/thread.hpp>

// STL
#include <cmath>
#include <vector>
#include <string>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// HOPE
#include "high_res_timer.h"
#include "z_growing.h"
#include "transform.h"
#include "utilities.h"
#include "pose_estimation.h"


enum data_type{SYN, POINT_CLOUD, TUM_SINGLE, TUM_LIST};


namespace hope {

  struct PlaneSegmentResults
  {
    size_t n_plane;

    /// Container for storing final results
    vector<Cloud_XYZ::Ptr> hull_results;
    vector<Cloud_XYZ::Ptr> point_results;
    vector<vector<float> > plane_params; // z area

    /// Container for storing the largest plane
    Cloud_XYZ::Ptr max_hull;
    Cloud_XYZ::Ptr max_points;
    float max_z;

    /// Container of plane id
  };

  class PlaneSegment
  {
  public:

    /** Base class for extracting planes in point cloud
     *
     * @param th_xy Clustering threshold for points in x-y plane
     * @param th_z Clustering threshold in z direction
     */
    PlaneSegment(float th_xy, float th_z);

    /**
     * Get all planes being parallel with the X-Y plane of the given cloud's coordinate frame.
     * @param cloud
     */
    PlaneSegmentResults getHorizontalPlanes(const Cloud_XYZ::Ptr& cloud, bool verbose = false);

  protected:
    data_type type_;

    /// Parameters of HOPE
    float th_xy_; // Resolution in XY direction
    float th_z_; // Resolution in Z direction

    /// Calculated parameters
    double th_theta_;
    double th_angle_;
    double th_norm_;

    /// Intermediate results
    // Source point cloud
    Cloud_XYZ::Ptr src_cloud_xyz_;

  private:
    // Normals of down sampling cloud
    Cloud_N::Ptr src_cloud_n_;
    // Normal filtered cloud index and corresponding cloud
    pcl::PointIndices::Ptr normal_fitted_indices_;

    Cloud_XYZ::Ptr normal_fitted_cloud_xyz_;
    Cloud_N::Ptr normal_fitted_cloud_n_;

    // Clustered points
    vector<float> plane_z_values_;
    vector<Cloud_XYZRGBN::Ptr> cloud_fit_parts_;
    vector<pcl::PointIndices> z_clustered_indices_list_;

    HighResTimer hst_;

    bool computeNormalAndFilter();

    /// Core process for finding planes
//    void findAllPlanesRANSAC(bool isOptimize, int maxIter, float disThresh, float omit);
//    void findAllPlanesRG(int norm_k, int num_n, float s_th, float c_th);

    /// Tool functions
    void reset();
    bool zClustering();
    bool getClustersMeanZ(bool verbose);
    void extractPlaneForEachZ(PlaneSegmentResults &results);
    bool getPlane(size_t id, float z_in, Cloud_XYZ::Ptr &plane_cloud);
    static bool getHull(Cloud_XYZ::Ptr cloud_xyz, Cloud_XYZ::Ptr &hull_xyz);
    bool gaussianImageAnalysis(size_t id);

    // Reconstruct mesh from point cloud
    static void poisson_reconstruction(const Cloud_XYZN::Ptr& point_cloud, pcl::PolygonMesh &mesh);
    static pcl::PolygonMesh mesh(const Cloud_XYZ::Ptr& point_cloud, const Cloud_N::Ptr& normals);
  };
}
//
//class PlaneSegment
//{
//public:
//
//  /** Class for extracting planes in point cloud
//   *
//   * @param th_xy Clustering threshold for points in x-y plane
//   * @param th_z Clustering threshold in z direction
//   * @param base_frame Optional, only used for point clouds obtained from ROS in real-time
//   * @param cloud_topic Optional, only used for point clouds obtained from ROS in real-time
//   */
//  PlaneSegment(data_type mode, float th_xy, float th_z, string base_frame = "", const string& cloud_topic = "");
//
//  void getHorizontalPlanes(Cloud_XYZRGB::Ptr cloud);
//
//  /// Container for storing final results
//  vector<Cloud_XYZRGB::Ptr> plane_results_;
//  vector<Cloud_XYZ::Ptr> plane_points_;
//  // Optional
//  vector<Cloud_XYZRGB::Ptr> plane_hull_;
//  vector<pcl::PolygonMesh> plane_mesh_;
//  vector<vector<float> > plane_coeff_;
//
//  /// Container for storing the largest plane
//  Cloud_XYZRGB::Ptr plane_max_result_;
//  Cloud_XYZ::Ptr plane_max_points_;
//  Cloud_XYZRGB::Ptr plane_max_hull_;
//  pcl::PolygonMesh plane_max_mesh_;
//  vector<float> plane_max_coeff_;
//
//  void setRPY(float roll, float pitch, float yaw);
//  void setQ(float qx, float qy, float qz, float qw);
//  void setT(float tx, float ty, float tz);
//
//protected:
//  data_type type_;
//
//
//private:
//  /// Predefined camera orientations if not using real data
//  // In Eular angle
//  float roll_;
//  float pitch_;
//  float yaw_;
//  // In Quaternion
//  float qx_, qy_, qz_, qw_;
//  // Camera position in the world coordinates frame
//  float tx_, ty_, tz_;
//  // Frame for point cloud to transfer
//  string base_frame_;
//
//  /// Supporting surface point number threshold
//  int global_size_temp_;
//  vector<vector<float> > global_coeff_temp_;
//  vector<int> global_id_temp_;
//
//  /// ROS stuff
//  ros::NodeHandle nh_;
//  // ROS pub-sub
//  ros::Subscriber sub_pointcloud_;
//  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
//
//  bool getSourceCloud();
//
//  /// Intermediate results
//  // Source cloud index from raw cloud (contain Nans)
//  pcl::PointIndices::Ptr src_z_inliers_;
//  // Source point cloud
//  Cloud_XYZRGB::Ptr src_rgb_cloud_;
//  Cloud_XYZ::Ptr src_mono_cloud_;
//
//  // Source cloud after down sampling
//  Cloud_XYZ::Ptr src_sp_mono_;
//  Cloud_XYZRGB::Ptr src_sp_rgb_;
//
//  // Normals of down sampling cloud
//  Cloud_N::Ptr src_normals_;
//  // Normal filtered cloud index and corresponding cloud
//  pcl::PointIndices::Ptr idx_norm_fit_;
//  Cloud_XYZ::Ptr cloud_norm_fit_mono_;
//  Cloud_N::Ptr cloud_norm_fit_;
//  // Clustered points
//  vector<float> plane_z_values_;
//  vector<Cloud_XYZRGBN::Ptr> cloud_fit_parts_;
//  vector<pcl::PointIndices> seed_clusters_indices_;
//
//  /// Tool objects
//  Transform *tf_;
//  HighResTimer hst_;
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//
//  bool computeNormalAndFilter();
//
//  /// Core process for finding planes
//  void findAllPlanes();
//  void findAllPlanesRANSAC(bool isOptimize, int maxIter, float disThresh, float omit);
//  void findAllPlanesRG(int norm_k, int num_n, float s_th, float c_th);
//
//  /// Tool functions
//  void reset();
//  void getMeanZofEachCluster(Cloud_XYZ::Ptr cloud_norm_fit_mono);
//  void extractPlaneForEachZ(Cloud_XYZ::Ptr cloud_norm_fit);
//  void zClustering(Cloud_XYZ::Ptr cloud_norm_fit_mono);
//  void getPlane(size_t id, float z_in, Cloud_XYZ::Ptr &cloud_norm_fit_mono);
//  bool gaussianImageAnalysis(size_t id);
//  int  checkSimilar(vector<float> coeff);
//  void setID();
//
//  /**
//   * @brief visualizeResult
//   * @param display_source display the source cloud
//   * @param display_raw display extracted points from planes
//   * @param display_err render the extracted points with their error towards the estimated model
//   * @param display_hull compose hulls with the extracted points
//   */
//  void visualizeResult(bool display_source, bool display_raw, bool display_err, bool display_hull);
//
//  // Reconstruct mesh from point cloud
//  void poisson_reconstruction(Cloud_XYZN::Ptr point_cloud, pcl::PolygonMesh &mesh);
//  pcl::PolygonMesh mesh(const Cloud_XYZ::Ptr point_cloud, Cloud_N::Ptr normals);
//
//  void visualizeProcess(Cloud_XYZRGB::Ptr cloud);
//  void setFeatures(float z_in, Cloud_XYZ::Ptr cluster);
//  void computeHull(Cloud_XYZRGB::Ptr cluster_2d_rgb);
//};
//
///**
// * Class for extracting horizontal planes from point cloud stream in real-time.
// */
//class PlaneSegmentRT
//{
//public:
//
//  /** Class for extracting planes in point cloud
//   *
//   * @param th_xy Clustering threshold for points in x-y plane
//   * @param th_z Clustering threshold in z direction
//   * @param nh The ROS node handle passed by outer function
//   * @param base_frame Optional, only used for point clouds obtained from ROS in real-time
//   * @param cloud_topic Optional, only used for point clouds obtained from ROS in real-time
//   */
//  PlaneSegmentRT(float th_xy, float th_z, ros::NodeHandle nh,
//    string base_frame = "", const string& cloud_topic = "");
//
//  ~PlaneSegmentRT() = default;
//
//  // If aggressively merge all planes with same height to one
//  bool aggressive_merge_;
//  void getHorizontalPlanes();
//
//  /// Container for storing the largest plane
//  Cloud_XYZ::Ptr max_plane_cloud_;
//  Cloud_XYZ::Ptr max_plane_contour_;
//  float max_plane_z_;
//
//  // The height of the object origin w.r.t. the base. This origin may not coincide
//  // with the mass centroid of the object, only used to infer its pose or ease
//  // the manipulation as it should be fixed with the object body.
//  float origin_height_;
//
//  std::vector<double> origin_heights_;
//
//  // Extracted objects' pose
//  geometry_msgs::PoseArray on_top_object_poses_;
//
//  // Extracted objects' categories
//  std::vector<int> on_top_object_categories_;
//
//private:
//  string base_frame_;
//
//  /// Supporting surface point number threshold
//  int max_plane_points_num_;
//  float min_height_;
//  float max_height_;
//  vector<int> global_id_temp_;
//
//  /// ROS stuff
//  ros::NodeHandle nh_;
//  dynamic_reconfigure::Server<hope::hopeConfig> config_server_;
//  ros::ServiceServer extract_on_top_server_;
//
//  ros::Subscriber cloud_subscriber_;
//  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
//  void configCallback(hope::hopeConfig &config, uint32_t level);
//  bool extractOnTopCallback(hope::ExtractObjectOnTop::Request &req,
//                            hope::ExtractObjectOnTop::Response &res);
//
//  ros::Publisher plane_cloud_publisher_;
//  ros::Publisher max_plane_cloud_publisher_;
//  ros::Publisher max_plane_hull_publisher_;
//  ros::Publisher on_plane_obj_publisher_;
//
//  void getSourceCloud();
//
//  /// Intermediate results
//  // Source cloud index from raw cloud (contain Nans)
//  pcl::PointIndices::Ptr src_z_inliers_;
//
//  // Source point cloud
//  Cloud_XYZ::Ptr src_mono_cloud_;
//
//  // Source cloud after down sampling
//  Cloud_XYZ::Ptr src_dsp_mono_;
//
//  // Normals of down sampling cloud
//  Cloud_N::Ptr src_normals_;
//
//  // Normal filtered cloud index and corresponding cloud
//  pcl::PointIndices::Ptr idx_norm_fit_;
//  Cloud_XYZ::Ptr cloud_norm_fit_mono_;
//  Cloud_N::Ptr cloud_norm_fit_;
//
//  // Clustered points
//  vector<float> plane_z_values_;
//  vector<pcl::PointIndices> seed_clusters_indices_;
//
//  /// Tool objects
//  Transform *tf_;
//  HighResTimer hst_;
//  PoseEstimation *pe_;
//
//  // object pcd file path, used when detect mesh type object
//  string object_model_path_;
//
//  void computeNormalAndFilter();
//
//  /// Core process for finding planes
//  void findAllPlanes();
//
//  /// Tool functions
//  void reset();
//  void getMeanZofEachCluster(Cloud_XYZ::Ptr cloud_norm_fit_mono);
//  void extractPlaneForEachZ(Cloud_XYZ::Ptr cloud_norm_fit);
//  void zClustering(const Cloud_XYZ::Ptr& cloud_norm_fit_mono);
//  void getPlane(size_t id, float z_in, Cloud_XYZ::Ptr &cloud_norm_fit_mono);
//  bool gaussianImageAnalysis(size_t id);
//
//  void visualizeResult();
//
//  /**
//   * In the real time mode, we could extract the clusters on top of the max
//   * plane with this function.
//   * @param type Geometric type of the object, could be cylinder; box; mesh.
//   * @param do_cluster Whether divide upper cloud into clusters.
//   */
//  bool postProcessing(bool do_cluster, string type);
//  void computeHull(Cloud_XYZ::Ptr cluster_2d, Cloud_XYZ::Ptr &cluster_hull);
//};

#endif // PLANE_SEGMENT_H
