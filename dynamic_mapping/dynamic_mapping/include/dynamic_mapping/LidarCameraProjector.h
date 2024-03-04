#ifndef DYNAMIC_MAPPING_LIDAR_CAMERA_PROJECTOR_H_
#define DYNAMIC_MAPPING_LIDAR_CAMERA_PROJECTOR_H_

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>

class LidarCameraProjector {
 public:
  void projectPtsToCameraFrame(const Eigen::Matrix3Xd& pts, const Eigen::Isometry3d& transform);
  std::vector<int> filterVisiblePoints(const Eigen::Matrix3Xd& pclPts, cv::Size maskShape);
  Eigen::Matrix3Xd getProjectedPoints() { return projectedPoints_; }
  Eigen::Matrix3Xd getPixelPoints() { return pixelPoints_; }
  Eigen::Matrix3Xd getVisiblePoints() { return visiblePoints_; }
  void setCameraProjectionMatrix(const Eigen::Matrix<double, 3, 4>& cameraProjection) { cameraProjection_ = cameraProjection; }

 private:
  Eigen::Matrix<double, 3, 4> cameraProjection_;
  Eigen::Matrix3Xd projectedPoints_;
  Eigen::Matrix3Xd pixelPoints_;
  Eigen::Matrix3Xd visiblePoints_;
};

#endif