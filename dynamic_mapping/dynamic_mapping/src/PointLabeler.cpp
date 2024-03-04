#include "dynamic_mapping/PointLabeler.h"
#include <open3d/geometry/PointCloud.h>
#include <opencv2/core/eigen.hpp>
#include <unordered_map>

Eigen::Matrix4Xd PointLabeler::labelPoints(const Eigen::Matrix3Xd& pts, const cv::Mat& segMask, const Eigen::Isometry3d& lidarToCamera,
                                           const bool isNewMask, std::vector<int>& labelPtsIdx) {
  projector_.projectPtsToCameraFrame(pts, lidarToCamera);
  labelPtsIdx = projector_.filterVisiblePoints(pts, segMask.size());
  Eigen::Matrix3Xi pixelPts = projector_.getPixelPoints().cast<int>();
  Eigen::Matrix3Xd visiblePts = projector_.getVisiblePoints();

  Eigen::RowVectorXd labels(pixelPts.cols());
  for (int i = 0; i < pixelPts.cols(); i++) {
    labels(i) = segMask.at<cv::Vec3b>(cv::Point(pixelPts(0, i), pixelPts(1, i)))[2];
  }
  Eigen::Matrix4Xd labelPoints(4, visiblePts.cols());
  labelPoints << visiblePts, labels;
  if (isNewMask) {
    labelledPoints_ = labelPoints;
    hasNewLabels_ = true;
  }
  return labelPoints;
}

// TODO: optimize labelling -> preallocate, use index vectors to avoid loops (?)
std::vector<Eigen::Matrix4Xd> PointLabeler::getLabeledClusters(const Eigen::Matrix3Xd& pts, int clusterPtsThreshold) {
  std::vector<Eigen::Matrix4Xd> result;
  open3d::geometry::PointCloud pcd = eigenToOpen3d(pts);
  std::vector<int> clusters = pcd.ClusterDBSCAN(0.6, 5);
  std::vector<int> uniqueClusters = getUnique(clusters);

  for (auto id : uniqueClusters) {
    std::vector<int> indices = getClusterIndices(clusters, id);
    Eigen::RowVectorXd labels(indices.size());

    if (id == -1 || indices.size() < clusterPtsThreshold) {
      // noise, or cluster too small
      continue;
    }

    std::vector<int> labelIndices;
    Eigen::Matrix3Xd clusterPts = pts(Eigen::all, indices);
    if (hasNewLabels_) {
      auto labelPoints = labelledPoints_(Eigen::seq(0, 2), Eigen::all);
      for (auto pt : clusterPts.colwise()) {
        for (int i = 0; i < labelPoints.cols(); ++i) {
          if ((labelPoints.col(i) == pt)) {
            // cluster point is labelled
            labelIndices.push_back(i);
            break;
          }
        }
      }
      if (labelIndices.size() == indices.size()) {
        // entire cluster has labels
        adjustClusterLabels(labels, labelIndices);
      } else {
        labels.setConstant(254);
      }
    } else {
      labels.setConstant(255);
    }
    Eigen::Matrix4Xd cluster(4, clusterPts.cols());
    cluster << clusterPts, labels;
    result.push_back(cluster);
  }
  hasNewLabels_ = false;
  return result;
}
void PointLabeler::adjustClusterLabels(Eigen::RowVectorXd& labels, const std::vector<int>& labelIndices) const {
  Eigen::RowVectorXd clusterLabel = labelledPoints_(3, labelIndices);
  std::vector<double> stdLabels;
  stdLabels.resize(clusterLabel.size());
  Eigen::RowVectorXd::Map(stdLabels.data(), clusterLabel.size()) = clusterLabel;
  std::unordered_map<double, int> labelCount = getOccurencePerElement(stdLabels);
  auto maxLabel = std::max_element(labelCount.begin(), labelCount.end(),
                                   [](const std::pair<double, int>& a, const std::pair<double, int>& b) { return a.second < b.second; });
  double ratio = static_cast<double>(maxLabel->second) / static_cast<double>(clusterLabel.size());
  if (ratio >= 0.4 && ratio < 1.0) {
    labels.setConstant(maxLabel->first);
  } else {
    labels = clusterLabel;
  }
}
