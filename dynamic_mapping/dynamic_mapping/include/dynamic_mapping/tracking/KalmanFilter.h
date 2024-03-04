#ifndef DYNAMIC_MAPPING_TRACKING_KALMAN_FILTER_H_
#define DYNAMIC_MAPPING_TRACKING_KALMAN_FILTER_H_

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/Dense>
#include "dynamic_mapping/tracking/GaussianState.h"

template <size_t DimState, size_t DimMeas>
class KalmanFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  template <size_t Dim>
  using TimeVariantMatrix = std::function<Eigen::Matrix<double, Dim, Dim>(double)>;
  using State = GaussianState<DimState>;
  typedef typename Eigen::Matrix<double, DimMeas, 1> MeasurementVector;
  typedef typename Eigen::Matrix<double, DimState, DimState> StateMatrix;
  typedef typename Eigen::Matrix<double, DimMeas, DimMeas> MeasurementMatrix;
  typedef typename Eigen::Matrix<double, DimMeas, DimState> MeasurementStateMatrix;
  typedef typename Eigen::Matrix<double, DimState, DimMeas> StateMeasurementMatrix;
  /**
   * @brief Create a Kalman Filter with specified matrices
   *
   * @param A System Dynamics
   * @param H State Observation Matrix
   * @param Q Process Noise Covariance
   * @param R Measurement Noise Covariance
   * @param P0 Initial State Covariance
   */
  KalmanFilter(const StateMatrix& A, const MeasurementStateMatrix& H, const TimeVariantMatrix<DimState>& Q,
               const TimeVariantMatrix<DimMeas>& R);
  KalmanFilter(const TimeVariantMatrix<DimState>& A, const MeasurementStateMatrix& H, const TimeVariantMatrix<DimState>& Q,
               const TimeVariantMatrix<DimMeas>& R);
  void init(double t0, const State& x0);
  void init(const State& x0);

  State predict(double t);
  void predict(const State& x, double dt, State& x_predict, MeasurementMatrix& S) const;

  State update(const MeasurementVector& z);
  MeasurementVector predictMeasurement(const State& x) { return H_ * x.state; };
  State getPrediction() { return x_predict_; };
  MeasurementMatrix getInnovationCovariance() { return S_; };
  State getState() { return x_; }

 private:
  double t_ = 0.f;
  StateMatrix I_;
  MeasurementStateMatrix H_;
  StateMeasurementMatrix K_;
  TimeVariantMatrix<DimState> A_, Q_;
  TimeVariantMatrix<DimMeas> R_;
  State x_predict_, x_;
  MeasurementMatrix S_;

  bool initialized_ = false;
};
#include "dynamic_mapping/tracking/KalmanFilter_impl.hpp"
#endif