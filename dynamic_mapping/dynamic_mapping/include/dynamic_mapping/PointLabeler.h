#ifndef DYNAMIC_MAPPING_POINT_LABELER_H_
#define DYNAMIC_MAPPING_POINT_LABELER_H_

#include <Eigen/Core>
#include <vector>
#include "dynamic_mapping/LidarCameraProjector.h"
#include "dynamic_mapping/helpers.h"

class PointLabeler {
 public:
  PointLabeler(LidarCameraProjector& projector) : projector_(projector) {}
  Eigen::Matrix4Xd labelPoints(const Eigen::Matrix3Xd& pts, const cv::Mat& segMask, const Eigen::Isometry3d& lidarToCamera,
                               const bool isNewMask, std::vector<int>& labelPtsIdx);
  std::vector<Eigen::Matrix4Xd> getLabeledClusters(const Eigen::Matrix3Xd& pts, int clusterPtsThreshold);

 private:
  LidarCameraProjector& projector_;
  Eigen::Matrix4Xd labelledPoints_;
  bool hasNewLabels_ = false;
  void adjustClusterLabels(Eigen::RowVectorXd& labels, const std::vector<int>& labelIndices) const;
};
#endif