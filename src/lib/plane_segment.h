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

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

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

// HOPE
#include "fetch_rgbd.h"
#include "high_res_timer.h"
#include "z_growing.h"
#include "transform.h"
#include "utilities.h"

using namespace std;
using namespace cv;

enum data_type{REAL, SYN, POINT_CLOUD, TUM_SINGLE, TUM_LIST};

class HopeResult
{
public:
  HopeResult();
  
  /// Container for storing final results
  vector<PointCloud::Ptr> plane_results_;
  vector<PointCloud::Ptr> plane_points_;
  vector<PointCloud::Ptr> plane_hull_;
  vector<pcl::PolygonMesh> plane_mesh_;
  vector<vector<float> > plane_params_; // z area
  
  /// Container for storing the largest plane
  PointCloud::Ptr plane_max_result_;
  PointCloud::Ptr plane_max_points_;
  PointCloud::Ptr plane_max_hull_;
  pcl::PolygonMesh plane_max_mesh_;
  vector<float> plane_max_param_;
  
  /// Container of plane id
  
};

class PlaneSegment
{
public:
  PlaneSegment(string base_frame, float th_xy, float th_z);

  inline void setMode(data_type type) {type_ = type;}

  void getHorizontalPlanes(PointCloud::Ptr cloud);
  
  /// Container for storing final results
  vector<PointCloud::Ptr> plane_results_;
  vector<PointCloudMono::Ptr> plane_points_;
  vector<PointCloud::Ptr> plane_hull_;
  vector<pcl::PolygonMesh> plane_mesh_;
  vector<vector<float> > plane_coeff_;

  /// Container for storing the largest plane
  PointCloud::Ptr plane_max_result_;
  PointCloudMono::Ptr plane_max_points_;
  PointCloud::Ptr plane_max_hull_;
  pcl::PolygonMesh plane_max_mesh_;
  vector<float> plane_max_coeff_;  

  void setRPY(float roll, float pitch, float yaw);
  void setQ(float qx, float qy, float qz, float qw);
  void setT(float tx, float ty, float tz);

protected:
  data_type type_;

  
private:
  /// Predefined camera orientations if not using real data
  // In Eular angle
  float roll_;
  float pitch_;
  float yaw_;
  // In Quaternion
  float qx_, qy_, qz_, qw_;
  // Camera position in the world coordinates frame
  float tx_, ty_, tz_;
  // Frame for point cloud to transfer
  string base_frame_;
  
  /// Supporting surface point number threshold
  int global_size_temp_;
  vector<vector<float> > global_coeff_temp_;
  vector<int> global_id_temp_;
  
  /// ROS stuff
  ros::NodeHandle nh_;
  image_transport::ImageTransport pub_it_;
  // ROS pub-sub
  ros::Subscriber sub_pointcloud_;
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  ros::Publisher pub_max_plane_;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_max_mesh_;
  template <typename PointTPtr>
  void publishCloud(PointTPtr cloud, ros::Publisher pub);
  bool getSourceCloud();
  
  /// Intermediate results
  // Source cloud index from raw cloud (contain Nans)
  pcl::PointIndices::Ptr src_z_inliers_;
  // Source point cloud
  PointCloud::Ptr src_rgb_cloud_;
  PointCloudMono::Ptr src_mono_cloud_;

  // Source cloud after down sampling
  PointCloudMono::Ptr src_sp_cloud_;
  PointCloud::Ptr src_sp_rgb_;

  // Normals of down sampling cloud
  NormalCloud::Ptr src_normals_;
  // Normal filtered cloud index and corresponding cloud
  pcl::PointIndices::Ptr idx_norm_fit_;
  PointCloudMono::Ptr cloud_norm_fit_mono_;
  // Clustered points
  vector<float> plane_z_values_;
  vector<PointCloudRGBN::Ptr> cloud_fit_parts_;
  vector<pcl::PointIndices> seed_clusters_indices_;
  
  /// Tool objects
  FetchRGBD *fi_;
  Transform *tf_;
  Utilities *utl_;
  HighResTimer hst_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  
  /// Core process for finding planes
  void findAllPlanes();
  void findPlaneWithPCL();
  
  /// Tool functions
  void reset();
  void getMeanZofEachCluster(PointCloudMono::Ptr cloud_norm_fit_mono);
  void extractPlaneForEachZ(PointCloudMono::Ptr cloud_norm_fit);
  void zClustering(PointCloudMono::Ptr cloud_norm_fit_mono);
  void getPlane(size_t id, float z_in, PointCloudMono::Ptr &cloud_norm_fit_mono);
  bool errorAnalyse(float z, PointCloudMono::Ptr cloud_in, PointCloud::Ptr &cloud_out, bool fix_z);
  int checkSimiliar(vector<float> coeff);
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
};

#endif // PLANE_SEGMENT_H
