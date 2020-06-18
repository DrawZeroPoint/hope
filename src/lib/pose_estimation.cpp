#include "pose_estimation.h"

using namespace std;

PoseEstimation::PoseEstimation(float dsp_th) :
  object_cloud_(new PointCloudN),
  object_features_(new PointCloudFPFH)
{
  dsp_th_ = dsp_th;
  object_model_path_ = "/home/dzp/model.pcd";
  int ok = pcl::io::loadPCDFile<PointN>(object_model_path_, *object_cloud_);
  if (ok < 0) {
    has_object_model_ = false;
  } else {
    Utilities::downSampling(object_cloud_, object_cloud_, dsp_th_, dsp_th_);
    Utilities::estimateNormals(object_cloud_, object_cloud_, dsp_th_);
    Utilities::estimateFPFH(object_cloud_, object_features_, dsp_th_);
    has_object_model_ = true;
  }
}

bool PoseEstimation::estimate(PointCloudN::Ptr scene_cloud, Eigen::Matrix4f &trans) {
  if (!has_object_model_) return false;

  Utilities::downSampling(scene_cloud, scene_cloud, dsp_th_, dsp_th_);
  Utilities::estimateNormals(scene_cloud, scene_cloud, dsp_th_);

  PointCloudFPFH::Ptr scene_features(new PointCloudFPFH);
  Utilities::estimateFPFH(scene_cloud, scene_features, dsp_th_);

  pcl::io::savePCDFile("/home/dzp/scene.pcd", *scene_cloud);
  pcl::io::savePCDFile("/home/dzp/obj.pcd", *object_cloud_);

  bool ok = Utilities::alignmentWithFPFH(object_cloud_, object_features_,
                                         scene_cloud, scene_features, trans, dsp_th_);

  cerr << ok << endl;
  printf("    | %6.3f %6.3f %6.3f | \n", trans(0,0), trans(0,1), trans(0,2));
  printf("R = | %6.3f %6.3f %6.3f | \n", trans(1,0), trans(1,1), trans(1,2));
  printf("    | %6.3f %6.3f %6.3f | \n", trans(2,0), trans(2,1), trans(2,2));
  cerr << trans(0,0) << trans(0,1) << trans(0,2) << endl;
  return ok;
}