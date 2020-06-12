#ifndef FETCH_RGBD_H
#define FETCH_RGBD_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Header.h>

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

#include <math.h>
#include <vector>
#include <string>

using namespace std;

class FetchRGBD
{
public:
  explicit FetchRGBD(string depth_topic, const string& rgb_topic = "", const string& camera_info_topic = "");
  
  void fetchRGBDInfo(cv_bridge::CvImagePtr &rgb, cv_bridge::CvImagePtr &depth,
                     sensor_msgs::CameraInfo &info);
  void fetchRGBD(cv_bridge::CvImagePtr &rgb, cv_bridge::CvImagePtr &depth);
  void fetchDepth(cv_bridge::CvImagePtr &depth);
  
private:
  ros::NodeHandle nh_;
  boost::shared_ptr<image_transport::ImageTransport> rgb_it_;
  boost::shared_ptr<image_transport::ImageTransport> depth_it_;

  bool use_rgb_;
  bool use_camera_info_;
  image_transport::SubscriberFilter sub_rgb_filter_;
  image_transport::SubscriberFilter sub_depth_filter_;

  image_transport::ImageTransport it_rgb_;
  image_transport::ImageTransport it_depth_;
  image_transport::Subscriber rgb_sub_;
  image_transport::Subscriber depth_sub_;

  void initRGBDInfoCb(const string& rgb_topic, const string& depth_topic, const string& camera_info_topic);
  //  void initDepthCb(const string& depth_topic);
  void initRGBDCb(const string &rgb_topic, const string &depth_topic);

  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
  
  typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
  SyncPolicy;
  
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> synchronizer_;
  
  // Image temp
  cv_bridge::CvImagePtr rgb_ptr_;
  cv_bridge::CvImagePtr depth_ptr_;
  // Image info
  sensor_msgs::CameraInfo cam_info_;
  
  void RGBDInfoCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                        const sensor_msgs::ImageConstPtr& depth_msg,
                        const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

  // void depthCallback(const sensor_msgs::ImageConstPtr &depth_msg);
  void RGBDCallback(const sensor_msgs::ImageConstPtr &rgb_msg, const sensor_msgs::ImageConstPtr &depth_msg);

  void depthCallback(const sensor_msgs::ImageConstPtr &depth_msg);

  void initDepthCb(const string &depth_topic);
};

#endif // FETCH_RGBD_H
