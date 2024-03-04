#include "dynamic_mapping/LidarCameraProjector.h"
#include <Eigen/Core>

void LidarCameraProjector::projectPtsToCameraFrame(const Eigen::Matrix3Xd& pts, const Eigen::Isometry3d& transform) {
  Eigen::Matrix4Xd homogenousPts = pts.colwise().homogeneous();
  const Eigen::Matrix4d& homogeneousTransform = transform.matrix();
  Eigen::Matrix<double, 3, 4> projection = cameraProjection_ * homogeneousTransform;
  projectedPoints_ = projection * homogenousPts;
}

std::vector<int> LidarCameraProjector::filterVisiblePoints(const Eigen::Matrix3Xd& pclPts, cv::Size maskShape) {
  Eigen::ArrayXXd u = projectedPoints_.row(0).array() / projectedPoints_.row(2).array();
  Eigen::ArrayXXd v = projectedPoints_.row(1).array() / projectedPoints_.row(2).array();
  Eigen::ArrayXXd z = projectedPoints_.row(2).array();
  int h = maskShape.height;
  int w = maskShape.width;
  auto inliers = (u >= 0) && (u < w) && (v >= 0) && (v < h) && (z > 0);
  std::vector<int> inlierIdx;
  for (int i = 0; i < inliers.size(); i++) {
    if (inliers(i)) {
      inlierIdx.push_back(i);
      // Delete column from projected points
    }
  }
  auto ptsInCamera = projectedPoints_(Eigen::all, inlierIdx);
  visiblePoints_ = pclPts(Eigen::all, inlierIdx);
  pixelPoints_ = ptsInCamera.array().rowwise() / ptsInCamera.row(2).array();
  pixelPoints_.row(2) = z;
  return inlierIdx;
}