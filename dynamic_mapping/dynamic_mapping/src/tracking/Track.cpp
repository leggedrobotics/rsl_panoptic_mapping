#include <utility>

#include "dynamic_mapping/helpers.h"
#include "dynamic_mapping/tracking/Track.h"

Track::Track(const GaussianState<9>& initialState, const FilterParameters& params, double timestamp, std::string uuid)
    : uuid_(std::move(uuid)) {
  kf_ = std::make_unique<Filter>(params.A, params.H, params.Q, params.R);
  kf_->init(timestamp, initialState);
}

std::ostream& operator<<(std::ostream& os, const Track& t) {
  os << "State: \n"
     << t.kf_->getState().state << "\nCovariance Trace:\n"
     << t.kf_->getState().covar.trace() << "\nLabel: " << t.getLabel() << "\n";
  return os;
}
int Track::getLabel() const {
  if (labelCounts_.empty()) {
    return 255;
  }
  auto maxIt = std::max_element(labelCounts_.begin(), labelCounts_.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
  return maxIt->first;
}
void Track::addState(const Filter::State& state, double timestamp) {
  double dt = 1;
  Eigen::Vector3d dx;
  if (!states_.empty()) {
    auto lastState = (--states_.end());
    dt = timestamp - lastState->first;
    dx = (state.state.tail(3) - lastState->second.state.tail(3));
  }

  states_.emplace(timestamp, state);
  double velocity = Eigen::Vector3d(state.state[1], state.state[3], state.state[5]).norm();
  meanVelocity_(velocity);

  if (dt != 0 && !dx.isZero()) {
    Eigen::Vector3d boxVelocity = dx / dt;
    extentVelocityBuffer_.push_back(boxVelocity);
    // meanBoxExtentVelocity_(boxVelocity);
  }
}

Eigen::Vector3d Track::getMeanBoxExtentVelocity() const {
  Eigen::Vector3d sum;
  for (const auto& vec : extentVelocityBuffer_) {
    sum += vec;
  }
  sum /= extentVelocityBuffer_.size();
  return sum;
}
