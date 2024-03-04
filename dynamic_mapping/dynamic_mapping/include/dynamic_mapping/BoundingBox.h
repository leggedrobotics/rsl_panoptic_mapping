#ifndef DYNAMIC_MAPPING_CPP_BOUNDINGBOX_H
#define DYNAMIC_MAPPING_CPP_BOUNDINGBOX_H
#include <open3d/geometry/BoundingVolume.h>
#include <Eigen/Core>
#include <utility>

struct BoundingBox {
  Eigen::Vector3d center;
  Eigen::Vector3d extent;
  double velocity;
  Eigen::Vector3d extentVelocity;
  int label;
  std::string trackUuid;
  BoundingBox(Eigen::Vector3d center, Eigen::Vector3d extent, double velocity, Eigen::Vector3d extentVelocity, int label,
              std::string trackUuid)
      : center(std::move(center)),
        extent(std::move(extent)),
        velocity(velocity),
        extentVelocity(std::move(extentVelocity)),
        label(label),
        trackUuid(std::move(trackUuid)) {}
  open3d::geometry::AxisAlignedBoundingBox getAxisAlignedBoundingBox(double scale = 1.0) const {
    Eigen::Vector3d minBound = center - ((extent / 2) * scale);
    Eigen::Vector3d maxBound = center + ((extent / 2) * scale);
    // limit the size of the bounding box to avoid too large boxes 1 meters
    maxBound = maxBound.cwiseMin(center + Eigen::Vector3d(2, 2, 2));
    minBound = minBound.cwiseMax(center - Eigen::Vector3d(2, 2, 2));
    return {minBound, maxBound};
  }
};
#endif  // DYNAMIC_MAPPING_CPP_BOUNDINGBOX_H
