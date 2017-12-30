#ifndef PLANE_SEGMENT_H
#define PLANE_SEGMENT_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>

//PCL
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

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>

#include <pcl/visualization/cloud_viewer.h>

#include <math.h>
#include <vector>
#include <string>

#include "fetch_rgbd.h"
#include "high_res_timer.h"
#include "transform.h"
#include "utilities.h"

using namespace std;
using namespace cv;

class PlaneSegment
{
public:
  PlaneSegment(bool use_real_data, string base_frame, float base_to_ground);
  
  void setParams(int dataset_type, float roll, float pitch, 
                 float tx, float ty, float tz, float qx, float qy, float qz, float qw);
  
  void getHorizontalPlanes(PointCloud::Ptr cloud);
  
  // Container for storing result planes
  vector<pcl::ModelCoefficients::Ptr> plane_coeff_;
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_hull_;
  // Container for storing the largest plane (except the ground)
  pcl::ModelCoefficients::Ptr plane_max_coeff_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_max_hull_;
  
private:
  bool use_real_data_;
  int dataset_type_;
  
  // Predefined camera orientations if not using real data
  // In RPY
  float roll_;
  float pitch_;
  // In Quaternion
  float qx_, qy_, qz_, qw_;
  // Camera position in the world coordinate
  float tx_, ty_, tz_;
  
  ros::NodeHandle nh_;
  image_transport::ImageTransport pub_it_;
  
  // Source point cloud and its inliers after z filter
  PointCloudMono::Ptr src_mono_cloud_;
  PointCloud::Ptr src_rgb_cloud_;
  pcl::PointIndices::Ptr src_z_inliers_;
  
  // Subscribe for real point cloud data
  ros::Subscriber sub_pointcloud_;
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  
  FetchRGBD *fi_;
  // The transform object can't be shared between Classes
  Transform *m_tf_;
  // Frame for point cloud to transfer
  string base_frame_;
  
  ros::Publisher pub_max_plane_;
  ros::Publisher pub_cloud_;
  
  // Target table approximate height
  float table_height_;
  
  // height max error 
  float th_height_;
  // Table area threshold
  float th_area_;
  
  // Distance between base_frame and ground
  float base_link_above_ground_;
  
  // Graspable area center xy in base_link frame
  float grasp_area_x_;
  float grasp_area_y_;
  float tolerance_;
  // Distance between object center and base
  float z_offset_;
  
  float global_area_temp_;
  float global_height_temp_;
  
  vector<float> planeZVector_;
  
  // Core process for finding planes
  void findAllPlanes();
  
  // Tool functions
  void calRegionGrowing(PointCloudRGBN::Ptr cloud_in, 
                        pcl::PointCloud<pcl::Normal>::Ptr normals, 
                        vector<pcl::PointIndices> &clusters);
  
  void getMeanZofEachCluster(vector<pcl::PointIndices> indices_in, 
                             PointCloudRGBN::Ptr cloud_in);
  
  /**
   * @brief extractPlaneForEachZ
   * Merge clouds which from the same plane with equal z value
   * @param cloud_in source cloud
   */
  void extractPlaneForEachZ(PointCloudRGBN::Ptr cloud_norm_fit);
  
  float getCloudMeanZ(PointCloudMono::Ptr cloud_in);
  
  /**
   * @brief extractPlane extract points which have similar z value
   * @param z_in target z value
   * @param cloud_in source point cloud
   */
  void extractPlane(float z_in, PointCloudRGBN::Ptr cloud_norm_fit);
  
  
  void visualizeResult();
  boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer;
  
  // For publishing and receiving point cloud
  template <typename PointTPtr>
  void publishCloud(PointTPtr cloud, ros::Publisher pub);
  bool getSourceCloud();
  
  HighResTimer hst_;
};

#endif // PLANE_SEGMENT_H
