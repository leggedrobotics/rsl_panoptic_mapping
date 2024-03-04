#include "dynamic_mapping/tracking/KalmanFilter.h"
template <size_t DimState, size_t DimMeas>
KalmanFilter<DimState, DimMeas>::KalmanFilter(const StateMatrix& A, const MeasurementStateMatrix& H, const TimeVariantMatrix<DimState>& Q,
                                              const TimeVariantMatrix<DimMeas>& R) {
  KalmanFilter([&](double dt) { return A; }, H, Q, R);
}
template <size_t DimState, size_t DimMeas>
KalmanFilter<DimState, DimMeas>::KalmanFilter(const TimeVariantMatrix<DimState>& A, const MeasurementStateMatrix& H,
                                              const TimeVariantMatrix<DimState>& Q, const TimeVariantMatrix<DimMeas>& R)
    : A_(A), H_(H), Q_(Q), R_(R), x_predict_(), I_(DimState, DimState) {
  I_.setIdentity();
}
template <size_t DimState, size_t DimMeas>
void KalmanFilter<DimState, DimMeas>::init(double t0, const State& x0) {
  t_ = t0;
  init(x0);
}
template <size_t DimState, size_t DimMeas>
void KalmanFilter<DimState, DimMeas>::init(const State& x0) {
  x_ = x0;
  initialized_ = true;
}
template <size_t DimState, size_t DimMeas>
typename KalmanFilter<DimState, DimMeas>::State KalmanFilter<DimState, DimMeas>::predict(double t) {
  if (!initialized_) {
    throw std::runtime_error("Not initialized!");
  }
  double dt = t - t_;
  predict(x_, dt, x_predict_, S_);
  x_ = x_predict_;
  t_ = t;
  return x_predict_;
}

template <size_t DimState, size_t DimMeas>
void KalmanFilter<DimState, DimMeas>::predict(const State& x, double dt, State& x_predict, MeasurementMatrix& S) const {
  StateMatrix A = A_(dt);
  StateMatrix Q = Q_(dt);
  MeasurementMatrix R = R_(dt);
  x_predict.state = A * x.state;
  x_predict.covar = A * x.covar * A.transpose() + Q;
  S = H_ * x_predict.covar * H_.transpose() + R;
}

template <size_t DimState, size_t DimMeas>
typename KalmanFilter<DimState, DimMeas>::State KalmanFilter<DimState, DimMeas>::update(const MeasurementVector& z) {
  if (!initialized_) {
    throw std::runtime_error("Not initialized!");
  }
  K_ = x_.covar * H_.transpose() * S_.inverse();
  x_.state = x_.state + K_ * (z - H_ * x_.state);
  x_.covar = x_.covar - K_ * S_ * K_.transpose();
  return x_;
}