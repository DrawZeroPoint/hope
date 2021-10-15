//
// Created by smart on 2021/10/14.
//

#ifndef SRC_PLANE_SEGMENT_ROS_H
#define SRC_PLANE_SEGMENT_ROS_H

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <hope/hopeConfig.h>
#include <hope/ExtractObjectOnTop.h>

// STL
#include <mutex>

// HOPE
#include "plane_segment.h"


namespace hope {

/**
 * Class for extracting horizontal planes from ROS point cloud topic.
 */
  class PlaneSegmentROS : public hope::PlaneSegment
  {
  public:

    /** Class for extracting planes in point cloud
     *
     * @param nh The ROS node handle passed by outer function
     * @param th_xy Clustering threshold for points in x-y plane
     * @param th_z Clustering threshold in z direction
     * @param base_frame Optional, only used for point clouds obtained from ROS in real-time
     * @param cloud_topic Optional, only used for point clouds obtained from ROS in real-time
     */
    PlaneSegmentROS(
        const ros::NodeHandle& nh,
        float th_xy,
        float th_z,
        double min_depth,
        double max_depth,
        double min_height,
        double max_height,
        string base_frame = "",
        const string& cloud_topic = "",
        const string& imu_topic = ""
    );

    ~PlaneSegmentROS() = default;

    // If aggressively merge all planes with same height to one
    bool aggressive_merge_;

    /// Container for storing the largest plane
    PlaneSegmentResults results_;

    // The height of the object origin w.r.t. the base. This origin may not coincide
    // with the mass centroid of the object, only used to infer its pose or ease
    // the manipulation as it should be fixed with the object body.
    float origin_height_;

    std::vector<double> origin_heights_;

    // Extracted objects' pose
    geometry_msgs::PoseArray on_top_object_poses_;

    // Extracted objects' categories
    std::vector<int> on_top_object_categories_;

  private:
    string base_frame_;

    double min_depth_;
    double max_depth_;

    /// Supporting surface point number threshold
    double min_height_;
    double max_height_;

    /// ROS stuff
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<hope::hopeConfig> config_server_;
    ros::ServiceServer extract_on_top_server_;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    void configCallback(hope::hopeConfig &config, uint32_t level);
    bool extractOnTopCallback(hope::ExtractObjectOnTop::Request &req,
                              hope::ExtractObjectOnTop::Response &res);

    void publishResults();

    ros::Subscriber cloud_subscriber_;

    ros::Publisher plane_cloud_publisher_;
    ros::Publisher max_plane_cloud_publisher_;
    ros::Publisher max_plane_hull_publisher_;
    ros::Publisher on_plane_obj_publisher_;

    /// Tool objects
    Transform *tf_;
    PoseEstimation *pe_;

    // object pcd file path, used when detect mesh type object
    string object_model_path_;

    std::mutex mutex_;

    /**
     * In the real time mode, we could extract the clusters on top of the max
     * plane with this function.
     * @param type Geometric type of the object, could be cylinder; box; mesh.
     * @param do_cluster Whether divide upper cloud into clusters.
     */
    bool postProcessing(bool do_cluster, const string& type);
  };

}



#endif //SRC_PLANE_SEGMENT_ROS_H
