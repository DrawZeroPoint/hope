#include "pose_estimation.h"

using namespace std;

PoseEstimation::PoseEstimation() :
  object_cloud_(new PointCloudN),
  object_features_(new PointCloudFPFH)
{
  object_model_path_ = "/home/dzp/model.pcd";
  int ok = pcl::io::loadPCDFile<PointN>(object_model_path_, *object_cloud_);
  if (ok < 0) {
    cerr << "Error loading object model from " << object_model_path_ << endl;
    has_object_model_ = false;
  } else {
    Utilities::downSampling(object_cloud_, object_cloud_, 0.005f, 0.005f);
    Utilities::estimateFPFH(object_cloud_, object_features_);
    has_object_model_ = true;
  }
}

bool PoseEstimation::estimate(PointCloudN::Ptr scene_cloud, Eigen::Matrix4f &trans) {
  if (!has_object_model_) return false;

  Utilities::estimateNormals(scene_cloud, scene_cloud);

  PointCloudFPFH::Ptr scene_features(new PointCloudFPFH);
  Utilities::estimateFPFH(scene_cloud, scene_features);

  bool ok = Utilities::alignmentWithFPFH(object_cloud_, object_features_, scene_cloud, scene_features, trans);
  return ok;
}