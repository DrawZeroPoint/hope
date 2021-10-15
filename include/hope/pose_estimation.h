#ifndef SRC_POSE_ESTIMATION_H
#define SRC_POSE_ESTIMATION_H

#include "utilities.h"


class PoseEstimation
{
public:
  PoseEstimation(float dsp_th = 0.005f);

  bool estimate(Cloud_XYZN::Ptr scene, Eigen::Matrix4f &trans, bool verbose = false);

private:
  float dsp_th_;

  bool has_object_model_;
  std::string object_model_path_;

  Cloud_XYZN::Ptr object_cloud_;
  PointCloudFPFH::Ptr object_features_;
};


#endif //SRC_POSE_ESTIMATION_H
