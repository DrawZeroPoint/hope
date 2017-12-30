#include "fetch_rgbd.h"

using namespace image_transport;

FetchRGBD::FetchRGBD()
{
  initRGBDCallback();
}

void FetchRGBD::fetchRGBD(cv_bridge::CvImagePtr &rgb, 
                          cv_bridge::CvImagePtr &depth,
                          sensor_msgs::CameraInfo &info)
{
  while (ros::ok()) {
    if (rgb_ptr_ != NULL && depth_ptr_ != NULL)
      break;
    ros::spinOnce();
    ros::Duration(0.005).sleep();
  }
  rgb = rgb_ptr_;
  depth = depth_ptr_;
  info = cam_info_;
}

void FetchRGBD::initRGBDCallback()
{
  rgb_it_.reset(new ImageTransport(nh_));
  depth_it_.reset(new ImageTransport(nh_));
  
  sub_rgb_.subscribe(*rgb_it_, "/camera/rgb/image_rect_color", 5, TransportHints("compressed"));
  sub_depth_.subscribe(*depth_it_, "/camera/depth/image_rect", 5, TransportHints("compressedDepth"));
  sub_camera_info_.subscribe(nh_, "/camera/depth/camera_info", 5);
  
  synchronizer_.reset(new Synchronizer(SyncPolicy(5), sub_rgb_, sub_depth_, sub_camera_info_));
  synchronizer_->registerCallback(boost::bind(&FetchRGBD::rgbdCallback, this, _1, _2, _3));
}

void FetchRGBD::rgbdCallback(const sensor_msgs::ImageConstPtr &rgb_msg, 
                             const sensor_msgs::ImageConstPtr &depth_msg, 
                             const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
{
  rgb_ptr_ = cv_bridge::toCvCopy(rgb_msg);
  depth_ptr_ = cv_bridge::toCvCopy(depth_msg);
  cam_info_ = *camera_info_msg;
}
