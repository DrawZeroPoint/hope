#ifndef PLANE_SEGMENT_H
#define PLANE_SEGMENT_H

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <hope/Subsections.h>
#include <peanut_ods/ObjectCandidateArray.h>
#include <peanut_ods/PlanarCandidate.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/surface/poisson.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

// STL
#include <math.h>
#include <vector>
#include <string>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// HOPE
#include "z_growing.h"
#include "transform.h"
#include "utilities.h"

// viz 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

enum data_type{REAL, SYN, POINT_CLOUD, TUM_SINGLE, TUM_LIST};

struct Params {
  double x_dim = 0.0;
  double y_dim = 0.0;
  double z_min = 0.0;
  double z_max = 0.0;
  double area_min = 0.0;
  double area_max = 0.0;
  double xy_resolution = 0.05;
  double z_resolution = 0.02;
  bool viz = true;
  std::string base_frame = "mobile_base_link";
  std::string cloud_topic = "/oil/perception/head_camera/cloud";
};


class PlaneSegment
{
public:
  PlaneSegment(Params params, ros::NodeHandle nh);

  inline void setMode(data_type type) {type_ = type;}

  void getHorizontalPlanes();
  
  /// Container for storing final results
  vector<PointCloud::Ptr> plane_results_;
  vector<PointCloudMono::Ptr> plane_points_;
  
  // Convex hull
  vector<PointCloudMono::Ptr> plane_hull_;
  vector<pcl::PolygonMesh> plane_mesh_;
  vector<vector<float> > plane_coeff_;
  vector<vector<geometry_msgs::Point>> convex_hull_pts_; 

  /// Container for storing the largest plane
  PointCloud::Ptr plane_max_result_;
  PointCloudMono::Ptr plane_max_points_;
  PointCloudMono::Ptr plane_max_hull_;
  pcl::PolygonMesh plane_max_mesh_;
  vector<float> plane_max_coeff_;  

protected:
  data_type type_;

private:
  /// Predefined camera orientations if not using real data
  // In Eular angle
  const double x_dim_{};
  const double y_dim_{};
  double min_area_;
  double max_area_;
  double z_min_;
  double z_max_;
  float roll_;
  float pitch_;
  float yaw_;

  // Frame for point cloud to transfer
  string base_frame_;
  
  /// Supporting surface point number threshold
  int global_size_temp_;
  vector<vector<float> > global_coeff_temp_;
  vector<int> global_id_temp_;
  hope::Subsections subsections_;
  
  /// ROS stuff
  ros::NodeHandle nh_;
  geometry_msgs::PolygonStamped polygon_array_;
  unsigned int min_cluster_size_; 

  // ROS pub-sub
  ros::Subscriber sub_pointcloud_;
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  ros::Publisher pub_max_plane_;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_max_mesh_;
  ros::Publisher polygon_pub_;
  ros::Publisher subsections_pub_;
  ros::Publisher polygon_marker_pub_;
  ros::Publisher convex_hull_pub_;
  ros::Publisher convex_hull_marker_pub_;
  ros::Publisher ods_planar_candidate_pub_;

  visualization_msgs::MarkerArray polygon_markers_;
  visualization_msgs::MarkerArray convex_hull_markers_; 

  template <typename PointTPtr>
  void publishCloud(PointTPtr cloud, ros::Publisher pub);
  bool getSourceCloud();
  
  /// Intermediate results
  // Source cloud index from raw cloud (contain Nans)
  pcl::PointIndices::Ptr src_z_inliers_;
  
  // Source point cloud
  PointCloud::Ptr src_rgb_cloud_;
  PointCloudMono::Ptr src_mono_cloud_;
  ros::Time last_cloud_time_; 
  ros::Time last_cloud_msg_time_;
  double cloud_time_threshold_; 

  // Source cloud after down sampling
  PointCloudMono::Ptr src_sp_mono_;
  PointCloud::Ptr src_sp_rgb_;

  // Normals of down sampling cloud
  NormalCloud::Ptr src_normals_;
  
  // Normal filtered cloud index and corresponding cloud
  pcl::PointIndices::Ptr idx_norm_fit_;
  PointCloudMono::Ptr cloud_norm_fit_mono_;
  NormalCloud::Ptr cloud_norm_fit_;
  
  // Clustered points
  vector<float> plane_z_values_;
  vector<PointCloudRGBN::Ptr> cloud_fit_parts_;
  vector<pcl::PointIndices> seed_clusters_indices_;
  
  /// Tool objects
  Transform *tf_;
  Utilities *utl_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  bool viz_;

  void computeNormalAndFilter();
  
  /// Core process for finding planes
  void findAllPlanes();
  void findAllPlanesRANSAC(bool isOptimize, int maxIter, float disThresh, float omit);
  void findAllPlanesRG(int norm_k, int num_n, float s_th, float c_th);
  
  /// Tool functions
  void reset();
  void getMeanZofEachCluster(PointCloudMono::Ptr cloud_norm_fit_mono);
  void extractPlaneForEachZ(PointCloudMono::Ptr cloud_norm_fit);
  void zClustering(PointCloudMono::Ptr cloud_norm_fit_mono);
  void getPlane(size_t id, float z_in, PointCloudMono::Ptr &cloud_norm_fit_mono);
  bool gaussianImageAnalysis(size_t id);
  int  checkSimiliar(vector<float> coeff);
  void setID();

  /**
   * @brief visualizeResult
   * @param display_source display the source cloud
   * @param display_raw display extracted points from planes
   * @param display_err render the extracted points with their error towards the estimated model
   * @param display_hull compose hulls with the extracted points
   */
  void visualizeResult(bool display_source, bool display_raw, bool display_err, bool display_hull);
  
  // Reconstruct mesh from point cloud
  void poisson_reconstruction(NormalPointCloud::Ptr point_cloud, 
                              pcl::PolygonMesh &mesh);
  pcl::PolygonMesh mesh(const PointCloudMono::Ptr point_cloud, NormalCloud::Ptr normals);
  
  void visualizeProcess(PointCloud::Ptr cloud);
  void setFeatures(float z_in, PointCloudMono::Ptr cluster);

  // Visualization and Convex hull
  void computeHull(PointCloudMono::Ptr cluster_2d_rgb);
  void pubishConvexHullCandidates();
  void addCloudMarkers(const PointCloudMono::Ptr cloud, const int id, visualization_msgs::MarkerArray& m_array);
  
};

#endif // PLANE_SEGMENT_H
