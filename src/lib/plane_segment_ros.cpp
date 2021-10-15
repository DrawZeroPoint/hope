//
// Created by smart on 2021/10/14.
//

#include "hope/plane_segment_ros.h"

namespace hope {

  PlaneSegmentROS::PlaneSegmentROS(
      const ros::NodeHandle& nh,
      float th_xy,
      float th_z,
      double min_depth,
      double max_depth,
      double min_height,
      double max_height,
      string base_frame,
      const string& cloud_topic,
      const string& imu_topic
  ):
      PlaneSegment(th_xy, th_z, CLOUD_STREAM, false),
      nh_(nh),
      min_depth_(min_depth),
      max_depth_(max_depth),
      min_height_(min_height),
      max_height_(max_height),
      tf_(new Transform),
      pe_(new PoseEstimation(th_xy)),
      base_frame_(std::move(base_frame)),
      origin_height_(0.0f),
      aggressive_merge_(true)
  {
    // Register the callback for real point cloud data
    cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        cloud_topic, 1, &PlaneSegmentROS::cloudCallback, this
    );

    // Set up dynamic reconfigure callback
    dynamic_reconfigure::Server<hope::hopeConfig>::CallbackType f;

    f = boost::bind(&PlaneSegmentROS::configCallback, this, _1, _2);
    config_server_.setCallback(f);

    extract_on_top_server_ = nh_.advertiseService(
        "extract_object_on_top", &PlaneSegmentROS::extractOnTopCallback, this
    );

    // Detect table surface as an obstacle
    plane_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("planes", 1);
    max_plane_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("max_plane", 1);
    max_plane_hull_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("max_plane_hull", 1);
    on_plane_obj_publisher_ = nh_.advertise<geometry_msgs::PoseArray>("obj_poses", 1, true);
  }

  void PlaneSegmentROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (msg->data.empty()) {
      ROS_WARN("HoPE: Source point cloud is empty.");
      return;
    }
    Cloud_XYZ::Ptr src_temp(new Cloud_XYZ);
    Cloud_XYZ::Ptr src_clamped(new Cloud_XYZ);
    Cloud_XYZ::Ptr src_transformed(new Cloud_XYZ);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *src_temp);

    pcl::PointIndices::Ptr src_z_inliers(new pcl::PointIndices);
    Utilities::getCloudByZ(src_temp, src_z_inliers, src_clamped, min_depth_, max_depth_);

    if (!tf_->getTransform(base_frame_, msg->header.frame_id)) return;
    tf_->doTransform(src_clamped, src_transformed);
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      getHorizontalPlanes(src_transformed);  // --> results_
      publishResults();
    }
  }

  void PlaneSegmentROS::configCallback(hope::hopeConfig &config, uint32_t level) {
    min_height_ = config.min_height;
    max_height_ = config.max_height;
    min_depth_ = config.min_depth;
    max_depth_ = config.max_depth;
  }

  void PlaneSegmentROS::publishResults() {
    if (results_.n_plane <= 0) return;
    Utilities::publishCloud(results_.max_plane, max_plane_cloud_publisher_, base_frame_);
    Utilities::publishCloud(results_.max_plane_hull, max_plane_hull_publisher_, base_frame_);
  }

  bool PlaneSegmentROS::extractOnTopCallback(hope::ExtractObjectOnTop::Request &req,
                                             hope::ExtractObjectOnTop::Response &res)
  {
    string object_type = req.goal_id.id;
    ROS_INFO("HoPE Service: Received extract on top object %s call.", object_type.c_str());
    bool do_cluster;
    if (object_type == hope::ExtractObjectOnTop::Request::CYLINDER) {
      origin_height_ = req.origin_height;
      object_type = "cylinder";
      do_cluster = true;
    } else if (object_type == hope::ExtractObjectOnTop::Request::MESH) {
      object_type = "mesh";
      object_model_path_ = req.mesh_path;
      do_cluster = false;
    } else if (object_type == hope::ExtractObjectOnTop::Request::BOX) {
      origin_height_ = req.origin_height;
      object_type = "box";
      do_cluster = true;
    } else if (object_type == hope::ExtractObjectOnTop::Request::BOX_TOP) {
      origin_heights_ = req.origin_heights;
      object_type = "box_top";
      do_cluster = true;
    } else if (req.goal_id.id == "debug") {
      origin_height_ = req.origin_height;
      object_type = "cylinder";
      do_cluster = true;
      req.header.stamp = ros::Time::now();
    } else {
      ROS_ERROR("HoPE: Unknown object type given in goal_id.id: %s", object_type.c_str());
      res.result_status = res.FAILED;
      return false;
    }

    aggressive_merge_ = req.aggressive_merge;

    if (postProcessing(do_cluster, object_type)) {
      uint time_interval = on_top_object_poses_.header.stamp.sec - req.header.stamp.sec;
      if (time_interval > 2) {
        ROS_WARN("HoPE Service: Extract on top object failed due to lagging %u.", time_interval);
        res.result_status = res.SUCCEEDED;
        res.obj_poses = on_top_object_poses_;
        res.categories = on_top_object_categories_;
      } else if (time_interval < 0) {
        ROS_WARN("HoPE service: Extract on top object failed due to looking into past %u.", time_interval);
        res.result_status = res.FAILED;
      } else {
        ROS_INFO("HoPE Service: Extract on top object succeeded.");
        res.result_status = res.SUCCEEDED;
        res.obj_poses = on_top_object_poses_;
        res.categories = on_top_object_categories_;
      }
    } else {
      res.result_status = res.FAILED;
    }
    return true;
  }

  bool PlaneSegmentROS::postProcessing(bool do_cluster, const string& type) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (Utilities::isPointCloudValid(results_.max_plane_hull)) {
      vector<Cloud_XYZ::Ptr> clusters;
      if (do_cluster) {
        if (!Utilities::getClustersUponPlane(src_cloud_xyz_, results_.max_plane_hull, clusters))
          return false;
        ROS_INFO("HoPE: Object clusters on plane #: %d", static_cast<int>(clusters.size()));
      } else {
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        Cloud_XYZ::Ptr upper_cloud(new Cloud_XYZ);
        Utilities::getCloudByZ(src_cloud_xyz_, indices, upper_cloud, results_.max_plane_z + 0.05f, 1000.0f);
        if (!Utilities::isPointCloudValid(upper_cloud)) {
          ROS_WARN("HoPE: No point cloud on top of the max plane.");
          return false;
        }
        clusters.push_back(upper_cloud);
      }

      on_top_object_poses_.poses.clear();
      on_top_object_categories_.clear();

      if (type != "mesh") {
        for (auto & cloud : clusters) {
          geometry_msgs::Pose pose;
          if (type == "cylinder") {
            if (!Utilities::getCylinderPose(cloud, pose, origin_height_))
              continue;
          } else if (type == "box") {
            try {
              if (!Utilities::getBoxPose(cloud, pose, origin_height_))
                continue;
            } catch (cv::Exception) {
              ROS_WARN("HoPE: Exception raised during getting box pose");
              continue;
            }
          } else if (type == "box_top") {
            try {
              int category;
              if (!Utilities::getBoxTopPose(cloud, pose, category, origin_heights_))
                continue;
              on_top_object_categories_.push_back(category);
            } catch (cv::Exception) {
              ROS_WARN("HoPE: Exception raised during getting box pose");
              continue;
            }
          } else {
            ROS_WARN("HoPE: Unknown object type %s", type.c_str());
            return false;
          }
          on_top_object_poses_.poses.push_back(pose);
        }
      } else {
        Cloud_XYZN::Ptr scene_cloud(new Cloud_XYZN);
        Utilities::convertCloudType(clusters[0], scene_cloud);
        Eigen::Matrix4f trans;
        pe_->estimate(scene_cloud, trans);
        Utilities::matrixToPoseArray(trans, on_top_object_poses_);
      }
      on_top_object_poses_.header.stamp = ros::Time::now();
      on_top_object_poses_.header.frame_id = base_frame_;
      on_plane_obj_publisher_.publish(on_top_object_poses_);
      return true;
    } else {
      ROS_WARN("HoPE: No valid plane for extracting objects on top.");
      return false;
    }
  }
}

