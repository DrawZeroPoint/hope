#include "hope/transform.h"

// Translation param from camera to base, in meter
float dx_camera_to_base = 0;
float dy_camera_to_base = 0;
float dz_camera_to_base = 1.0;

namespace hope {

  Transform::Transform() :
      tf_buffer_(),
      tf_listener_(tf_buffer_, nh_)
  {
    // Initialize node handler before tf_buffer is important
  }

  bool Transform::getTransform(const string& base_frame, const string& header_frame)
  {
    try {
      // Loop until getting the transform
      while (ros::ok()) {
        // Check if the transform from map to quad can be made right now
        if (tf_buffer_.canTransform(base_frame, header_frame, ros::Time(0))) {
          // Get the transform
          tf_handle_ = tf_buffer_.lookupTransform(base_frame, header_frame, ros::Time(0));
          return true;
        } else {
          ROS_WARN("HoPE: Transform from '%s' to '%s' does not exist.", base_frame.c_str(), header_frame.c_str());
        }
        ros::spinOnce();
      }
    }
      // Catch any exceptions that might happen while transforming
    catch (tf2::TransformException& ex) {
      ROS_ERROR("Exception transforming %s to %s: %s",
                base_frame.c_str(),
                header_frame.c_str(),
                ex.what());
      return false;
    }
  }

  void Transform::doTransform(const Cloud_XYZRGB::Ptr& cloud_in,
                              Cloud_XYZRGB::Ptr &cloud_out) const
  {
    geometry_msgs::Vector3 trans = tf_handle_.transform.translation;
    geometry_msgs::Quaternion rotate = tf_handle_.transform.rotation;

    Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Translation3d(trans.x, trans.y, trans.z)
                                                   * Eigen::Quaternion<double>(rotate.w, rotate.x, rotate.y, rotate.z);

    cloud_out->height = cloud_in->height;
    cloud_out->width  = cloud_in->width;
    cloud_out->is_dense = false;
    cloud_out->resize(cloud_out->height * cloud_out->width);

    Eigen::Vector3d point;
    size_t i = 0;
    for (Cloud_XYZRGB::const_iterator pit = cloud_in->begin();
         pit != cloud_in->end(); ++pit) {
      point = t * Eigen::Vector3d(pit->x, pit->y, pit->z);
      cloud_out->points[i].x = static_cast<float>(point.x());
      cloud_out->points[i].y = static_cast<float>(point.y());
      cloud_out->points[i].z = static_cast<float>(point.z());
      cloud_out->points[i].r = cloud_in->points[i].r;
      cloud_out->points[i].g = cloud_in->points[i].g;
      cloud_out->points[i].b = cloud_in->points[i].b;
      ++i;
    }
  }

  void Transform::doTransform(const Cloud_XYZ::Ptr& cloud_in,
                              Cloud_XYZ::Ptr &cloud_out) const
  {
    geometry_msgs::Vector3 trans = tf_handle_.transform.translation;
    geometry_msgs::Quaternion rotate = tf_handle_.transform.rotation;

    Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Translation3d(trans.x, trans.y, trans.z)
                                                   * Eigen::Quaternion<double>(rotate.w, rotate.x, rotate.y, rotate.z);

    cloud_out->height = cloud_in->height;
    cloud_out->width  = cloud_in->width;
    cloud_out->is_dense = false;
    cloud_out->resize(cloud_out->height * cloud_out->width);

    Eigen::Vector3d point;
    size_t i = 0;
    for (Cloud_XYZ::const_iterator pit = cloud_in->begin();
         pit != cloud_in->end(); ++pit) {
      point = t * Eigen::Vector3d(pit->x, pit->y, pit->z);
      cloud_out->points[i].x = static_cast<float>(point.x());
      cloud_out->points[i].y = static_cast<float>(point.y());
      cloud_out->points[i].z = static_cast<float>(point.z());
      ++i;
    }
  }

/**
 * @brief Transform::doTransform
 * @param cloud_in
 * @param cloud_out
 * @param roll The Euler Angle is in radius
 * @param pitch When pointing through the axis with thumb of right hand,
 * @param yaw the other 4 finger direct to the positive direction of the angle
 */
  void Transform::doTransform(const Cloud_XYZRGB::Ptr& cloud_in, Cloud_XYZRGB::Ptr &cloud_out,
                              float roll, float pitch, float yaw)
  {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.transform.translation.x = dx_camera_to_base;
    transformStamped.transform.translation.y = dy_camera_to_base;
    transformStamped.transform.translation.z = dz_camera_to_base;
    tf2::Quaternion q2;
    // Notice the range of roll and pitch value
    q2.setRPY(roll, pitch, yaw);
    q2.setY(- q2.y());
    transformStamped.transform.rotation.x = q2.x();
    transformStamped.transform.rotation.y = q2.y();
    transformStamped.transform.rotation.z = q2.z();
    transformStamped.transform.rotation.w = q2.w();

    geometry_msgs::Vector3 trans = transformStamped.transform.translation;
    geometry_msgs::Quaternion rotate = transformStamped.transform.rotation;

    Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Translation3d(trans.x, trans.y, trans.z)
                                                * Eigen::Quaternion<double>(rotate.w, rotate.x, rotate.y, rotate.z);

    cloud_out->height = cloud_in->height;
    cloud_out->width  = cloud_in->width;
    cloud_out->is_dense = false;
    cloud_out->resize(cloud_out->height * cloud_out->width);

    Eigen::Vector3d point;
    size_t i = 0;
    for (Cloud_XYZRGB::const_iterator pit = cloud_in->begin();
         pit != cloud_in->end(); ++pit) {
      point = t * Eigen::Vector3d(pit->x, pit->y, pit->z);
      cloud_out->points[i].x = static_cast<float>(point.x());
      cloud_out->points[i].y = static_cast<float>(point.y());
      cloud_out->points[i].z = static_cast<float>(point.z());
      cloud_out->points[i].r = pit->r;
      cloud_out->points[i].g = pit->g;
      cloud_out->points[i].b = pit->b;
      ++i;
    }
  }

  void Transform::doTransform(const Cloud_XYZRGB::Ptr& cloud_in, Cloud_XYZRGB::Ptr &cloud_out,
                              float qx, float qy, float qz, float qw, float tx, float ty, float tz)
  {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.transform.translation.x = tx;
    transformStamped.transform.translation.y = ty;
    transformStamped.transform.translation.z = tz;

    transformStamped.transform.rotation.x = qx;
    transformStamped.transform.rotation.y = qy;
    transformStamped.transform.rotation.z = qz;
    transformStamped.transform.rotation.w = qw;

    geometry_msgs::Vector3 trans = transformStamped.transform.translation;
    geometry_msgs::Quaternion rotate = transformStamped.transform.rotation;

    Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Translation3d(trans.x, trans.y, trans.z)
                                                   * Eigen::Quaternion<double>(rotate.w, rotate.x, rotate.y, rotate.z);

    cloud_out->height = cloud_in->height;
    cloud_out->width  = cloud_in->width;
    cloud_out->is_dense = false;
    cloud_out->resize(cloud_out->height * cloud_out->width);

    Eigen::Vector3d point;
    size_t i = 0;
    for (Cloud_XYZRGB::const_iterator pit = cloud_in->begin();
         pit != cloud_in->end(); ++pit) {
      point = t * Eigen::Vector3d(pit->x, pit->y, pit->z);
      cloud_out->points[i].x = static_cast<float>(point.x());
      cloud_out->points[i].y = static_cast<float>(point.y());
      cloud_out->points[i].z = static_cast<float>(point.z());
      cloud_out->points[i].r = pit->r;
      cloud_out->points[i].g = pit->g;
      cloud_out->points[i].b = pit->b;
      ++i;
    }
  }

  void Transform::doTransform(pcl::PointXYZ p_in, pcl::PointXYZ &p_out)
  {
    geometry_msgs::Vector3 trans = tf_handle_.transform.translation;
    geometry_msgs::Quaternion rotate = tf_handle_.transform.rotation;

    Eigen::Transform<float,3,Eigen::Affine> t = Eigen::Translation3f(trans.x,
                                                                     trans.y,
                                                                     trans.z)
                                                * Eigen::Quaternion<float>(rotate.w, rotate.x, rotate.y, rotate.z);

    Eigen::Vector3f point;

    point = t * Eigen::Vector3f(p_in.x, p_in.y, p_in.z);
    p_out.x = point.x();
    p_out.y = point.y();
    p_out.z = point.z();
  }
}
