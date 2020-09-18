//
// Created by dzp on 2020/9/18.
//

#include "palletization.h"


Palletization::Palletization(ros::NodeHandle nh, string base_frame, float th_xy, float th_z)
    : nh_(nh), base_frame_(base_frame), th_grid_rsl_(th_xy), th_z_rsl_(th_z), tf_(new Transform)
{
  get_object_pose_server_ = nh_.advertiseService("get_object_info", &Palletization::getObjectInfoCb, this);
  object_pose_puber_ = nh_.advertise<geometry_msgs::PoseArray>("object_poses", 1, true);

  th_theta_ = th_z_rsl_ / th_grid_rsl_;
  th_norm_ = sqrt(1 / (1 + 2 * pow(th_theta_, 2)));
}

bool Palletization::getObjectInfoCb(hope::GetObjectPose::Request &req,
                                    hope::GetObjectPose::Response &res) {
  origin_heights_ = req.origin_heights;
  PointCloudMono::Ptr src_cloud(new PointCloudMono);
  pcl::fromROSMsg(req.points, *src_cloud);

  if (!Utilities::isPointCloudValid(src_cloud)) {
    ROS_ERROR("HoPE: Source cloud is empty.");
    res.result_status = res.FAILED;
    return true;
  }

  reset();

  if (!tf_->getTransform(base_frame_, req.points.header.frame_id)) return false;
  tf_->doTransform(src_cloud, src_mono_cloud_);

  Utilities::downSampling(src_mono_cloud_, src_dsp_mono_, th_grid_rsl_, th_z_rsl_);
  if (!Utilities::isPointCloudValid(src_dsp_mono_)) {
    ROS_ERROR("HoPE: Down sampled source cloud is empty.");
    res.result_status = res.FAILED;
    return true;
  }

  computeNormalAndFilter();
  zClustering(cloud_norm_fit_mono_); // -> seed_clusters_indices_
  if (seed_clusters_indices_.empty()) {
    ROS_ERROR("HoPE: Z growing got nothing.");
    res.result_status = res.FAILED;
    return true;
  }

  plane_z_values_.clear();
  plane_z_values_.insert(plane_z_values_.end(), req.origin_heights.begin(), req.origin_heights.end());
  extractPlaneForEachZ(cloud_norm_fit_mono_);

  int category;
  geometry_msgs::Pose pose;
  if (postProcessing(category, pose)) {
    res.pose = pose;
    res.category = category;
    res.result_status = res.SUCCEEDED;
  } else {
    res.result_status = res.FAILED;
  }

  return true;
}

void Palletization::reset()
{
  src_mono_cloud_.reset(new PointCloudMono);
  src_dsp_mono_.reset(new PointCloudMono);
  cloud_norm_fit_mono_.reset(new PointCloudMono);
  max_plane_cloud_.reset(new PointCloudMono);
  src_normals_.reset(new CloudN);
  idx_norm_fit_.reset(new pcl::PointIndices);

  plane_z_values_.clear();
  seed_clusters_indices_.clear();

  max_plane_points_num_ = 0;
}

void Palletization::computeNormalAndFilter()
{
  Utilities::estimateNorm(src_dsp_mono_, src_normals_, 1.01 * th_grid_rsl_);
  Utilities::getCloudByNorm(src_normals_, idx_norm_fit_, th_norm_);
  if (idx_norm_fit_->indices.empty()) {
    ROS_WARN("HoPE: No point fits the normal criteria");
    return;
  }
  Utilities::getCloudByInliers(src_dsp_mono_, cloud_norm_fit_mono_, idx_norm_fit_, false, false);
}

void Palletization::extractPlaneForEachZ(PointCloudMono::Ptr cloud_norm_fit)
{
  size_t id = 0;
  for (float & plane_z_value : plane_z_values_) {
    getPlane(id, plane_z_value, cloud_norm_fit);
    id++;
  }
}

void Palletization::getPlane(size_t id, float z_in, PointCloudMono::Ptr &cloud_norm_fit_mono)
{
  pcl::PointIndices::Ptr idx_seed(new pcl::PointIndices);
  idx_seed->indices = seed_clusters_indices_[id].indices;

  // Extract the plane points indexed by idx_seed
  PointCloudMono::Ptr cloud_z(new PointCloudMono);
  Utilities::getCloudByInliers(cloud_norm_fit_mono, cloud_z, idx_seed, false, false);

  // Update the data of the max plane detected
  if (cloud_z->points.size() > max_plane_points_num_) {
    max_plane_cloud_ = cloud_z;
    // Use convex hull to represent the plane patch
    max_plane_z_ = z_in;
    max_plane_points_num_ = cloud_z->points.size();
  }
}

void Palletization::zClustering(const PointCloudMono::Ptr& cloud_norm_fit_mono)
{
  ZGrowing zg;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
      (new pcl::search::KdTree<pcl::PointXYZ>);

  zg.setMinClusterSize(3);
  zg.setMaxClusterSize(INT_MAX);
  zg.setSearchMethod(tree);
  zg.setNumberOfNeighbours(8);
  zg.setInputCloud(cloud_norm_fit_mono);
  zg.setZThreshold(th_z_rsl_);

  zg.extract(seed_clusters_indices_);
}

bool Palletization::postProcessing(int& category, geometry_msgs::Pose& pose) {
  if (Utilities::isPointCloudValid(max_plane_cloud_)) {
    try {
      bool ok = Utilities::getBoxTopPose(max_plane_cloud_, pose, category, origin_heights_);
      if (ok) {
        // Extracted objects' pose
        geometry_msgs::PoseArray object_poses;
        object_poses.header.stamp = ros::Time::now();
        object_poses.header.frame_id = base_frame_;
        object_poses.poses.push_back(pose);
        object_pose_puber_.publish(object_poses);
        return true;
      } else {
        ROS_WARN("HoPE: Get box top pose failed");
        return false;
      }
    } catch (cv::Exception) {
      ROS_WARN("HoPE: Exception raised during getting box pose");
      return false;
    }
  } else {
    ROS_WARN("HoPE: Max plane not valid");
    return false;
  }
}