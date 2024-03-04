#ifndef DYNAMIC_MAPPING_TRACKING_DETECTION_H_
#define DYNAMIC_MAPPING_TRACKING_DETECTION_H_

#include <Eigen/Dense>
#include <boost/functional/hash.hpp>

struct Detection {
  Eigen::Vector3d centroid;
  Eigen::Vector3d extent;
  int label{};
  int id;
  double timestamp{};
  bool isMissedDetection = false;
  Detection(const Eigen::Vector3d& centroid, const Eigen::Vector3d& extent, int label, double timestamp)
      : centroid(centroid), extent(extent), label(label), timestamp(timestamp), isMissedDetection(false), id(id_++) {}
  Detection() : isMissedDetection(true), id(id_++) {}
  Eigen::Matrix<double, 6, 1> measurement() const {
    Eigen::Matrix<double, 6, 1> meas;
    meas << centroid, extent;
    return meas;
  }
  friend bool operator==(const Detection& lhs, const Detection& rhs) { return lhs.id == rhs.id; }

  friend bool operator<(const Detection& lhs, const Detection& rhs) { return lhs.id < rhs.id; }

  friend bool operator>(const Detection& lhs, const Detection& rhs) { return operator<(rhs, lhs); }

  friend bool operator<=(const Detection& lhs, const Detection& rhs) { return !(operator>(lhs, rhs)); }

  friend bool operator>=(const Detection& lhs, const Detection& rhs) { return !(operator<(lhs, rhs)); }

  friend bool operator!=(const Detection& lhs, const Detection& rhs) { return !(operator==(lhs, rhs)); }

  static size_t id_;
};

#endif