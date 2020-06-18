#ifndef SRC_POSE_ESTIMATION_H
#define SRC_POSE_ESTIMATION_H

#include "utilities.h"


class PoseEstimation
{
public:
  PoseEstimation();

  bool estimate(PointCloudN::Ptr scene, Eigen::Matrix4f &trans);

private:
  bool has_object_model_;
  std::string object_model_path_;

  PointCloudN::Ptr object_cloud_;
  PointCloudFPFH::Ptr object_features_;
};


#endif //SRC_POSE_ESTIMATION_H
