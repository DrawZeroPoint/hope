//
// Created by dzp on 2020/9/18.
//

#ifndef SRC_PALLETIZATION_H
#define SRC_PALLETIZATION_H


// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <hope/hopeConfig.h>
#include <hope/GetObjectPose.h>

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

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// HOPE
#include "high_res_timer.h"
#include "z_growing.h"
#include "transform.h"
#include "utilities.h"
#include "pose_estimation.h"


class Palletization {

public:
  Palletization(ros::NodeHandle nh, string base_frame, float th_xy, float th_z);
  ~Palletization() = default;

private:
  /// Params
  string base_frame_;
  float th_grid_rsl_;
  float th_z_rsl_;
  float th_theta_;
  float th_norm_;

  /// Object surface point number
  int max_plane_points_num_;

  /// Container for storing the largest plane
  Cloud_XYZ::Ptr max_plane_cloud_;
  float max_plane_z_;

  // The height of the object origin w.r.t. the base. This origin may not coincide
  // with the mass centroid of the object, only used to infer its pose or ease
  // the manipulation as it should be fixed with the object body.
  std::vector<double> origin_heights_;

  /// Tool objects
  Transform *tf_;

  /// ROS stuff
  ros::NodeHandle nh_;
  ros::ServiceServer get_object_pose_server_;

  ros::Publisher object_pose_puber_;

  /// Point clouds
  // Source point cloud
  Cloud_XYZ::Ptr src_mono_cloud_;

  // Source cloud after down sampling
  Cloud_XYZ::Ptr src_dsp_mono_;

  // Normals of down sampling cloud
  Cloud_N::Ptr src_normals_;

  // Normal filtered cloud index and corresponding cloud
  pcl::PointIndices::Ptr idx_norm_fit_;
  Cloud_XYZ::Ptr cloud_norm_fit_mono_;

  // Clustered points
  vector<float> plane_z_values_;
  vector<pcl::PointIndices> seed_clusters_indices_;

  bool getObjectInfoCb(hope::GetObjectPose::Request &req,
                       hope::GetObjectPose::Response &res);

  void computeNormalAndFilter();
  void reset();

  void extractPlaneForEachZ(Cloud_XYZ::Ptr cloud_norm_fit);
  void zClustering(const Cloud_XYZ::Ptr& cloud_norm_fit_mono);

  void getPlane(size_t id, float z_in, Cloud_XYZ::Ptr &cloud_norm_fit_mono);
  bool postProcessing(int& category, geometry_msgs::Pose& pose);
};


#endif //SRC_PALLETIZATION_H
