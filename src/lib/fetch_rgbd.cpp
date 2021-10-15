#include "hope/fetch_rgbd.h"

using namespace image_transport;

FetchRGBD::FetchRGBD(string depth_topic, const string& rgb_topic, const string& camera_info_topic) :
  it_rgb_(nh_),
  it_depth_(nh_)
{
  if (depth_topic.empty()) depth_topic = "/camera/depth/image";
  use_rgb_ = !rgb_topic.empty();
  use_camera_info_ = !camera_info_topic.empty();
  cerr << "using rgb: " << use_rgb_ << " use camera info " << use_camera_info_ << endl;

  if (use_rgb_ && use_camera_info_) {
    initRGBDInfoCb(rgb_topic, depth_topic, camera_info_topic);
  } else if (use_rgb_ && !use_camera_info_) {
    initRGBDCb(rgb_topic, depth_topic);
  } else if (!use_rgb_ && !use_camera_info_) {
    initDepthCb(depth_topic);
  }
}

void FetchRGBD::fetchRGBDInfo(cv_bridge::CvImagePtr &rgb,
                              cv_bridge::CvImagePtr &depth,
                              sensor_msgs::CameraInfo &info)
{
  rgb_ptr_ = nullptr;
  depth_ptr_ = nullptr;
  while (ros::ok()) {
    if (rgb_ptr_ != nullptr && depth_ptr_ != nullptr)
      break;
    ros::spinOnce();
    ros::Duration(0.001).sleep();
  }
  rgb = rgb_ptr_;
  depth = depth_ptr_;
  info = cam_info_;
}

void FetchRGBD::fetchRGBD(cv_bridge::CvImagePtr &rgb, cv_bridge::CvImagePtr &depth)
{
  rgb_ptr_ = nullptr;
  depth_ptr_ = nullptr;
  while (ros::ok()) {
    if (rgb_ptr_ != nullptr && depth_ptr_ != nullptr)
      break;
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  rgb = rgb_ptr_;
  depth = depth_ptr_;
}

void FetchRGBD::fetchDepth(cv_bridge::CvImagePtr &depth)
{
  depth_ptr_ = nullptr;
  while (ros::ok()) {
    if (depth_ptr_ != nullptr)
      break;
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  depth = depth_ptr_;
}

void FetchRGBD::initRGBDInfoCb(const string& rgb_topic, const string& depth_topic, const string& camera_info_topic)
{
  rgb_it_.reset(new ImageTransport(nh_));
  depth_it_.reset(new ImageTransport(nh_));
  
  sub_rgb_filter_.subscribe(*rgb_it_, rgb_topic, 5, TransportHints("compressed"));
  sub_depth_filter_.subscribe(*depth_it_, depth_topic, 5, TransportHints("compressedDepth"));
  sub_camera_info_.subscribe(nh_, camera_info_topic, 5);
  
  synchronizer_.reset(new Synchronizer(SyncPolicy(5), sub_rgb_filter_, sub_depth_filter_, sub_camera_info_));
  synchronizer_->registerCallback(boost::bind(&FetchRGBD::RGBDInfoCallback, this, _1, _2, _3));
}

void FetchRGBD::initRGBDCb(const string& rgb_topic, const string& depth_topic)
{
  rgb_it_.reset(new ImageTransport(nh_));
  depth_it_.reset(new ImageTransport(nh_));

  sub_rgb_filter_.subscribe(*rgb_it_, rgb_topic, 5);
  sub_depth_filter_.subscribe(*depth_it_, depth_topic, 5);

  synchronizer_.reset(new Synchronizer(SyncPolicy(5), sub_rgb_filter_, sub_depth_filter_));
  synchronizer_->registerCallback(boost::bind(&FetchRGBD::RGBDCallback, this, _1, _2));
}

void FetchRGBD::initDepthCb(const string &depth_topic)
{
  depth_sub_ = it_depth_.subscribe(depth_topic, 1, &FetchRGBD::depthCallback, this);
}

void FetchRGBD::RGBDInfoCallback(const sensor_msgs::ImageConstPtr &rgb_msg,
                                 const sensor_msgs::ImageConstPtr &depth_msg,
                                 const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
{
  rgb_ptr_ = cv_bridge::toCvCopy(rgb_msg);
  depth_ptr_ = cv_bridge::toCvCopy(depth_msg);
  cam_info_ = *camera_info_msg;
}

void FetchRGBD::RGBDCallback(const sensor_msgs::ImageConstPtr &rgb_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
  rgb_ptr_ = cv_bridge::toCvCopy(rgb_msg);
  depth_ptr_ = cv_bridge::toCvCopy(depth_msg);
}

void FetchRGBD::depthCallback(const sensor_msgs::ImageConstPtr &depth_msg)
{
  depth_ptr_ = cv_bridge::toCvCopy(depth_msg);  // encoding 32FC1
}
