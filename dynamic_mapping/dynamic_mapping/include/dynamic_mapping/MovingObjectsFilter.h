#ifndef DYNAMIC_MAPPING_CPP_MOVINGOBJECTSFILTER_H
#define DYNAMIC_MAPPING_CPP_MOVINGOBJECTSFILTER_H
#include <Eigen/Core>
#include <mutex>
#include <unordered_set>
#include "dynamic_mapping/BoundingBox.h"
#include "dynamic_mapping/LidarCameraProjector.h"
#include "dynamic_mapping/Parameters.h"
#include "dynamic_mapping/PointLabeler.h"
#include "dynamic_mapping/tracking/Tracker.h"

class MovingObjectsFilter {
 public:
  explicit MovingObjectsFilter(const ObjectFilterParameters& parameters);
  void filterObjects(const Eigen::Matrix3Xd& pointCloud, const Eigen::Matrix3Xd& noGroundPointCloud, double timestamp);

  void setCameraTransform(const Eigen::Isometry3d& tf) { toCamera_ = tf; };
  void updateCameraProjection(const Eigen::Matrix<double, 3, 4>& newProjection);
  void updateLabels(const Eigen::Matrix3Xd& noGroundPointCloud, const cv::Mat& segmentationMask, double timestamp);
  void setShovelFrame(const Eigen::Isometry3d& frame) { shovelFrame_ = frame; }

  std::vector<BoundingBox> getTrackBoxes() { return bboxes_; };
  std::vector<BoundingBox> getDetectionBoxes() { return detectionBoxes_; };
  std::vector<Eigen::Matrix4Xd> getLabelledClusters() { return labelledClusters_; };
  Eigen::Matrix4Xd getFilteredCloudStatic() { return outputCloudStatic_; };
  Eigen::Matrix4Xd getFilteredCloudDynamic() { return outputCloudDynamic_; };
  std::vector<Track> getTracks();

  LidarCameraProjector projector_;

 private:
  std::vector<Detection> generateDetections(const std::vector<Eigen::Matrix4Xd>& labelledClusters, double timestamp);
  std::vector<BoundingBox> detectionBoxes_;
  std::vector<BoundingBox> tracksToBoundingBoxes();
  Eigen::Matrix<double, 3, 4> cameraProjection_;
  std::unique_ptr<PointLabeler> labeler_;
  std::unique_ptr<Tracker> tracker_;
  Eigen::Isometry3d toCamera_;
  std::vector<BoundingBox> bboxes_;
  std::vector<Eigen::Matrix4Xd> labelledClusters_;
  std::mutex cameraParameterLock_, segMaskLock_;
  Eigen::Matrix4Xd outputCloudStatic_;
  Eigen::Matrix4Xd outputCloudDynamic_;
  cv::Mat currentSegMask_;
  void cropPointCloud(const Eigen::Matrix3Xd& cloud, const Eigen::Matrix3Xd& noGroundPointCloud, double scale,
                      Eigen::Matrix4Xd& cloudStatic, Eigen::Matrix4Xd& cloudDynamic);
  double boundingBoxMargin_;
  double velocityFilterThreshold_ = 0.5;
  int minimumClusterSize_;
  double shovelDistanceThreshold_;
  double extentVelocityThreshold_;
  bool strictSegmentation_;
  std::vector<int> labels_;
  std::unordered_set<std::string> movingTrackIds_;
  std::vector<Track> tracks_;
  Eigen::Isometry3d shovelFrame_;
  bool segmentOnGroundRemovedCloud_;
  std::vector<int> getInverseIndices(std::vector<size_t>& indices, const Eigen::Matrix3Xd& pointcloud) const;
};

#endif  // DYNAMIC_MAPPING_CPP_MOVINGOBJECTSFILTER_H
