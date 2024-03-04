#ifndef DYNAMIC_MAPPING_TRACKING_TRACK_H_
#define DYNAMIC_MAPPING_TRACKING_TRACK_H_

#include <Eigen/Dense>
#include <algorithm>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/uuid/uuid.hpp>
#include <memory>
#include <unordered_map>
#include "GaussianState.h"
#include "dynamic_mapping/helpers.h"
#include "dynamic_mapping/tracking/Detection.h"
#include "dynamic_mapping/tracking/KalmanFilter.h"

using boost::accumulators::accumulator_set;
using boost::accumulators::stats;
using boost::accumulators::tag::rolling_mean;

class Track {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using Filter = KalmanFilter<9, 6>;
  using RollingMean = accumulator_set<double, stats<rolling_mean>>;
  using VectorBuffer = boost::circular_buffer<Eigen::Vector3d>;

  struct FilterParameters {
    const Filter::TimeVariantMatrix<9> A;
    const Filter::MeasurementStateMatrix H;
    const Filter::TimeVariantMatrix<9> Q;
    const Filter::TimeVariantMatrix<6> R;
    FilterParameters(const Filter::TimeVariantMatrix<9>& A, const Filter::MeasurementStateMatrix& H, const Filter::TimeVariantMatrix<9>& Q,
                     const Filter::TimeVariantMatrix<6>& R)
        : A(A), H(H), Q(Q), R(R) {}
  };

  Track(const GaussianState<9>& initialState, const FilterParameters& params, double timestamp, std::string uuid);
  Track(const Track& other)
      : kf_(std::make_unique<Filter>(*other.kf_)),
        states_(other.states_),
        labelCounts_(other.labelCounts_),
        meanVelocity_(other.meanVelocity_),
        meanBoxExtentVelocity_(other.meanBoxExtentVelocity_),
        uuid_(other.uuid_){};

  Filter::State predict(double timestamp) { return kf_->predict(timestamp); }
  Filter::MeasurementVector predictMeasurement(const Filter::State& x) const { return kf_->predictMeasurement(x); };
  Filter::MeasurementMatrix innovationCovariance() const { return kf_->getInnovationCovariance(); };
  Filter::State correct(const Detection& detection) { return kf_->update(detection.measurement()); }
  Filter::State getPrediction() { return kf_->getPrediction(); }
  void addState(const Filter::State& state, double timestamp);

  Filter::State getState() const { return kf_->getState(); };
  Filter::StateMatrix getCovariance() const { return kf_->getState().covar; }
  Filter getKF() const { return *kf_; }
  void setLabel(int label) { labelCounts_[label] += 1; }
  int getLabel() const;
  std::map<double, Filter::State> getStates() { return states_; }
  std::string getUuid() const { return uuid_; }

  friend std::ostream& operator<<(std::ostream& os, const Track& t);
  double getMeanVelocity() const { return boost::accumulators::rolling_mean(meanVelocity_); }
  Eigen::Vector3d getMeanBoxExtentVelocity() const;
  // { return boost::accumulators::rolling_mean(meanBoxExtentVelocity_); }

 private:
  std::unique_ptr<Filter> kf_;
  std::map<double, Filter::State> states_;
  std::map<int, int> labelCounts_;
  RollingMean meanVelocity_ = RollingMean(boost::accumulators::tag::rolling_window::window_size = 3);
  RollingMean meanBoxExtentVelocity_ = RollingMean(boost::accumulators::tag::rolling_window::window_size = 3);
  VectorBuffer extentVelocityBuffer_ = VectorBuffer(3);
  std::string uuid_;
};

#endif