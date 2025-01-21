#include "dynamic_mapping/MovingObjectsFilter.h"
#include <open3d/geometry/BoundingVolume.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <iostream>

MovingObjectsFilter::MovingObjectsFilter(const ObjectFilterParameters& parameters)
    : boundingBoxMargin_(parameters.boundingBoxMargin),
      velocityFilterThreshold_(parameters.velocityFilterThreshold),
      minimumClusterSize_(parameters.minimumClusterSize),
      labels_(parameters.labels),
      shovelDistanceThreshold_(parameters.shovelDistanceThreshold),
      extentVelocityThreshold_(parameters.extentVelocityThreshold),
      strictSegmentation_(parameters.strictSegmentation),
      segmentOnGroundRemovedCloud_(parameters.segmentOnGroundRemovedCloud) {
  labeler_ = std::make_unique<PointLabeler>(projector_);
  tracker_ = std::make_unique<Tracker>(parameters.labels, parameters.missedDetectionDistance, parameters.deletionCovarThreshold,
                                       parameters.initiationCovarThreshold, parameters.minimumInitiationPoints);
}
void MovingObjectsFilter::filterObjects(const Eigen::Matrix3Xd& pointCloud, const Eigen::Matrix3Xd& noGroundPointCloud, double timestamp) {
  labelledClusters_ = labeler_->getLabeledClusters(noGroundPointCloud, minimumClusterSize_);
  std::vector<Detection> detections = generateDetections(labelledClusters_, timestamp);
  tracker_->track(detections);
  bboxes_ = tracksToBoundingBoxes();
  cropPointCloud(pointCloud, noGroundPointCloud, boundingBoxMargin_, outputCloudStatic_, outputCloudDynamic_);

}

std::vector<Detection> MovingObjectsFilter::generateDetections(const std::vector<Eigen::Matrix4Xd>& labelledClusters, double timestamp) {
  std::vector<Detection> detections;
  detectionBoxes_.clear();
  for (const auto& cluster : labelledClusters) {
    open3d::geometry::AxisAlignedBoundingBox aabb = getBoundingBox(cluster(Eigen::seq(0, 2), Eigen::placeholders::all));
    Eigen::RowVectorXd labels = cluster.row(3);
    std::vector<double> stdLabels;
    stdLabels.resize(labels.size());
    Eigen::RowVectorXd::Map(stdLabels.data(), labels.size()) = labels;
    std::unordered_map<double, int> labelCount = getOccurencePerElement(stdLabels);
    auto maxLabel = std::max_element(labelCount.begin(), labelCount.end(),
                                     [](const std::pair<double, int>& a, const std::pair<double, int>& b) { return a.second < b.second; });
    detectionBoxes_.emplace_back(aabb.GetCenter(), aabb.GetExtent(), 0.0, Eigen::Vector3d::Zero(), maxLabel->first, " ");
    detections.emplace_back(aabb.GetCenter(), aabb.GetExtent(), maxLabel->first, timestamp);
  }
  return detections;
}
void MovingObjectsFilter::updateCameraProjection(const Eigen::Matrix<double, 3, 4>& newProjection) {
  {
    std::lock_guard<std::mutex> guard{cameraParameterLock_};
    cameraProjection_ = newProjection;
    projector_.setCameraProjectionMatrix(cameraProjection_);
  }
}

std::vector<BoundingBox> MovingObjectsFilter::tracksToBoundingBoxes() {
  std::vector<BoundingBox> boxes;
  for (const auto& track : *tracker_->getTracks()) {
    if (track.second.getLabel() < 0) {
      continue;
    }
    Eigen::Vector3d center;
    Eigen::Vector3d dimensions;
    Eigen::Vector<double, 9> state = track.second.getState().state;
    center << state[0], state[2], state[4];
    dimensions << state(Eigen::seq(6, 8));
    double velocity = track.second.getMeanVelocity();
    Eigen::Vector3d extentVelocity = track.second.getMeanBoxExtentVelocity();
    BoundingBox bbox = BoundingBox(center, dimensions, velocity, extentVelocity, track.second.getLabel(), track.first);
    boxes.push_back(bbox);
  }
  return boxes;
}

std::vector<Track> MovingObjectsFilter::getTracks() {
  std::vector<Track> tracks;
  for (const auto& track : *tracker_->getTracks()) {
    if (track.second.getLabel() <= 0) {
      continue;
    }
    tracks.push_back(track.second);
  }
  return tracks;
}

void MovingObjectsFilter::cropPointCloud(const Eigen::Matrix3Xd& cloud, const Eigen::Matrix3Xd& noGroundPointCloud, double scale,
                                         Eigen::Matrix4Xd& cloudStatic, Eigen::Matrix4Xd& cloudDynamic) {
  std::vector<size_t> indicesStatic;
  std::vector<size_t> indicesDynamic;
  Eigen::Vector3d max, min;
  Eigen::Matrix3Xd cleanCloud = removeNans(cloud);
  indicesStatic.reserve(cleanCloud.cols());
  indicesDynamic.reserve(cleanCloud.cols());
  Eigen::RowVectorXd labels = Eigen::RowVectorXd::Zero(cleanCloud.cols());

  open3d::geometry::KDTreeFlann rawCloudTree(cleanCloud);
  if (segmentOnGroundRemovedCloud_) {
    for (int i = 0; i < noGroundPointCloud.cols(); i++) {
      bool outside = true;
      int originalIdx = -1;
      Eigen::Vector3d col = noGroundPointCloud.col(i);
      for (const auto& box : bboxes_) {
        bool isDropping =
            box.extentVelocity.z() > extentVelocityThreshold_ && (minDistanceToFrame(box, shovelFrame_) < shovelDistanceThreshold_);
        bool isMoving = (box.velocity >= velocityFilterThreshold_);
        bool movedBefore = (movingTrackIds_.find(box.trackUuid) != movingTrackIds_.end());
        bool isLabelToBeFiltered = std::find(labels_.begin(), labels_.end(), box.label) != labels_.end();
        // if (((isMoving || isDropping || movedBefore) && !strictSegmentation_) || isLabelToBeFiltered) {
        if (isLabelToBeFiltered || isDropping) {
          movingTrackIds_.insert(box.trackUuid);
        }
        auto aabb = box.getAxisAlignedBoundingBox(1.0);
        min = aabb.GetMinBound();
        max = aabb.GetMaxBound();
        if ((col.array() >= min.array()).all() && (col.array() <= max.array()).all()) {
          std::vector<int> index;
          std::vector<double> sqrDistance;
          rawCloudTree.SearchKNN(col, 5, index, sqrDistance);
          if (sqrt(sqrDistance[0]) <= 0.1) {
            outside = false;
            originalIdx = index[0];
            if (!isLabelToBeFiltered) {
              labels[originalIdx] = 0;
            } else {
              labels[originalIdx] = box.label;
            }
          }
        }
        //}
      }
      if (originalIdx >= 0) {
        if (outside) {
          indicesStatic.push_back(originalIdx);
        } else {
          indicesDynamic.push_back(originalIdx);
        }
      }
    }
  } else {
    for (int i = 0; i < cleanCloud.cols(); i++) {
      Eigen::Vector3d col = cleanCloud.col(i);
      bool outside = true;
      for (const auto& box : bboxes_) {
        bool isDropping =
            box.extentVelocity.z() > extentVelocityThreshold_ && (minDistanceToFrame(box, shovelFrame_) < shovelDistanceThreshold_);
        bool isMoving = (box.velocity >= velocityFilterThreshold_);
        bool movedBefore = (movingTrackIds_.find(box.trackUuid) != movingTrackIds_.end());
        bool isLabelToBeFiltered = std::find(labels_.begin(), labels_.end(), box.label) != labels_.end();
        // if (((isMoving || isDropping || movedBefore) && !strictSegmentation_) || isLabelToBeFiltered) {
        if (isLabelToBeFiltered || isDropping) {
          movingTrackIds_.insert(box.trackUuid);
        }

        auto aabb = box.getAxisAlignedBoundingBox(scale);
        min = aabb.GetMinBound();
        max = aabb.GetMaxBound();
        if ((col.array() >= min.array()).all() && (col.array() <= max.array()).all()) {
          outside = false;
          if (!isLabelToBeFiltered) {
            labels[i] = 32;
          } else {
            labels[i] = box.label;
          }
        }
        //}
      }
      if (outside) {
        indicesStatic.push_back(i);
      } else {
        indicesDynamic.push_back(i);
      }
    }
  }

  Eigen::Matrix4Xd labelledStaticCloud;
  Eigen::Matrix3Xd staticCloud;
  if (!indicesStatic.empty()) {
    staticCloud = cleanCloud(Eigen::all, indicesStatic);
  } else if (!indicesDynamic.empty()) {
    staticCloud = cleanCloud(Eigen::all, getInverseIndices(indicesDynamic, cleanCloud));
  } else {
    staticCloud = cleanCloud;
  }

  std::vector<int> indices;
  {
    std::lock_guard<std::mutex> guard{segMaskLock_}; // Issue below
    labelledStaticCloud = labeler_->labelPoints(staticCloud, currentSegMask_, toCamera_, false, indices);
  }

  cloudStatic.conservativeResize(4, staticCloud.cols());
  cloudStatic.topRows(3) = staticCloud;
  cloudStatic.bottomRows(1).setConstant(255);

  for (int i = 0; i < indices.size(); i++) {
    auto label = labelledStaticCloud(3, i);
    bool isLabelToBeFiltered = std::find(labels_.begin(), labels_.end(), label) != labels_.end();

    cloudStatic(3, indices[i]) = (isLabelToBeFiltered) ? 255 : label;
  }

  Eigen::Matrix4Xd concatCloud;
  if (!indicesDynamic.empty()) {
    concatCloud.conservativeResize(4, indicesDynamic.size());
    concatCloud.topRows(3) = cleanCloud(Eigen::all, indicesDynamic);
    concatCloud.bottomRows(1) = labels(indicesDynamic);
    cloudDynamic = concatCloud;
  } else {
    cloudDynamic = Eigen::Vector4d::Zero();
  };
}
std::vector<int> MovingObjectsFilter::getInverseIndices(std::vector<size_t>& indices, const Eigen::Matrix3Xd& pointcloud) const {
  Eigen::VectorXi linIdx = Eigen::VectorXi::LinSpaced(pointcloud.cols(), 0, pointcloud.cols() - 1);
  std::set<int> idx{linIdx.data(), linIdx.data() + linIdx.size()};
  std::set<int> dynIdx{indices.begin(), indices.end()};
  std::vector<int> inverseIndices;
  std::set_difference(idx.begin(), idx.end(), dynIdx.begin(), dynIdx.end(), std::back_inserter(inverseIndices));
  return inverseIndices;
}

void MovingObjectsFilter::updateLabels(const Eigen::Matrix3Xd& noGroundPointCloud, const cv::Mat& segmentationMask, double timestamp) {
  // std::cout<<"currentSegMask_ updated"<<std::endl;
  {
    std::lock_guard<std::mutex> guard{segMaskLock_};
    currentSegMask_ = segmentationMask;
  }
  {
    std::lock_guard<std::mutex> guard{cameraParameterLock_};
    std::vector<int> indices;
    labeler_->labelPoints(noGroundPointCloud, segmentationMask, toCamera_, true, indices);
  }
  labelledClusters_ = labeler_->getLabeledClusters(noGroundPointCloud, minimumClusterSize_);
  std::vector<Detection> detections = generateDetections(labelledClusters_, timestamp);
  tracker_->updateTrackLabels(detections);
}