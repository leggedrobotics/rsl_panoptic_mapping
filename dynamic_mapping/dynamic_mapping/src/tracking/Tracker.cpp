#include "dynamic_mapping/tracking/Tracker.h"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include "dynamic_mapping/helpers.h"
#include "dynamic_mapping/tracking/Hungarian.h"

Tracker::Tracker(const std::vector<int>& initiableLabels, const double& missedDistance, const double& covarTraceThreshold,
                 const double& initiationCovarTraceThreshold, const int& minimumMeasurementsForInitiation)
    : initiableLabels_(initiableLabels),
      missedDistance_(missedDistance),
      covarTraceThreshold_(covarTraceThreshold),
      initialTraceThreshold_(initiationCovarTraceThreshold),
      minPoints_(minimumMeasurementsForInitiation) {
  Track::Filter::MeasurementStateMatrix stateObservationMatrix;
  Track::Filter::StateMatrix priorStateCovariance;
  Track::Filter::TimeVariantMatrix<9> systemDynamics, processNoiseCovariance;
  stateObservationMatrix.fill(0);
  stateObservationMatrix(0, 0) = 1;
  stateObservationMatrix(1, 2) = 1;
  stateObservationMatrix(2, 4) = 1;
  stateObservationMatrix(3, 6) = 1;
  stateObservationMatrix(4, 7) = 1;
  stateObservationMatrix(5, 8) = 1;

  priorStateCovariance.fill(0);
  priorStateCovariance.diagonal() << 0, 100, 0, 100, 0, 100, 0, 0, 0;

  auto R = [&](double dt = 1.0) {
    Track::Filter::MeasurementMatrix measCovar;
    measCovar.fill(0);
    measCovar.diagonal() << 0.05, 0.05, 0.05, 0.075, 0.075, 0.075;
    return measCovar;
  };

  Track::Filter::StateMeasurementMatrix invMeasModel = stateObservationMatrix.completeOrthogonalDecomposition().pseudoInverse();
  P0_ = invMeasModel * R() * invMeasModel.transpose();
  P0_ = P0_ + priorStateCovariance;

  systemDynamics = [](double dt) {
    Track::Filter::StateMatrix A, T;
    A.setIdentity();
    T.fill(0);
    T.diagonal<1>() << dt, 0, dt, 0, dt, 0, 0, 0;
    Track::Filter::StateMatrix res = A + T;
    return res;
  };

  processNoiseCovariance = [](double dt) {
    Track::Filter::StateMatrix Q;
    Eigen::Matrix2d covar{{pow(dt, 3) / 3.f, pow(dt, 2) / 2.f}, {pow(dt, 2) / 2.f, dt}};
    Q.fill(0);
    Q.block<2, 2>(0, 0) = covar * 4;
    Q.block<2, 2>(2, 2) = covar * 4;
    Q.block<2, 2>(4, 4) = covar * 2;
    Q.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.2;
    return Q;
  };
  filterParams_ = std::make_unique<Track::FilterParameters>(systemDynamics, stateObservationMatrix, processNoiseCovariance, R);

  uuidGenerator_ = boost::uuids::random_generator();
}

void Tracker::track(const std::vector<Detection>& detections) {
  std::unordered_map<std::string, Detection> associations = associateDetections(tracks_, detections);
  std::vector<Detection> matchedDetections;
  for (const auto& a : associations) {
    if (!a.second.isMissedDetection) {
      tracks_.at(a.first).addState(tracks_.at(a.first).correct(a.second), a.second.timestamp);
      matchedDetections.push_back(a.second);
    } else {
      tracks_.at(a.first).addState(tracks_.at(a.first).getState(), detections[0].timestamp);
    }
  }
  std::vector<Detection> unmatchedDetections;
  std::copy_if(detections.begin(), detections.end(), std::back_inserter(unmatchedDetections), [&](const Detection& d) -> bool {
    return (std::find(matchedDetections.begin(), matchedDetections.end(), d) == matchedDetections.end());
  });

  deleteTracks(tracks_, covarTraceThreshold_);
  initiateTracks(unmatchedDetections);
}

std::unordered_map<std::string, Detection> Tracker::associateDetections(std::unordered_map<std::string, Track>& tracks,
                                                                        const std::vector<Detection>& detections,
                                                                        bool isLabelUpdate) const {
  std::unordered_map<std::string, Detection> associations;
  std::unordered_map<std::string, DetectionDict> hypotheses;
  if (isLabelUpdate) {
    hypotheses = generateTrackHypotheses(tracks, detections, detections[0].timestamp);
  } else {
    hypotheses = generateTrackHypotheses(tracks, detections);
  }
  std::vector<std::string> detectedTracks;
  for (const auto& h : hypotheses) {
    if (h.second.size() > 1) {
      // there is more than the null hypothesis
      detectedTracks.push_back(h.first);
    } else {
      associations.insert(std::pair<std::string, Detection>(h.first, h.second.begin()->second));
    }
  }
  if (detectedTracks.empty()) {
    // no hypotheses, can return
    return associations;
  }

  std::vector<std::vector<std::pair<double, Detection>>> hypothesisMatrix;
  hypothesisMatrix.resize(detectedTracks.size());
  for (int i = 0; i < detectedTracks.size(); i++) {
    std::vector<std::pair<double, Detection>> row;
    row.resize(detections.size() + detectedTracks.size());
    for (const auto& h : hypotheses[detectedTracks[i]]) {
      if (h.second.isMissedDetection) {
        row[detections.size() + i] = h;
      } else {
        int idx = std::find(detections.begin(), detections.end(), h.second) - detections.begin();
        row[idx] = h;
      }
    }
    hypothesisMatrix[i] = row;
  }
  Eigen::MatrixXd distanceMatrix;
  distanceMatrix.resize(hypothesisMatrix.size(), hypothesisMatrix[0].size());
  for (int i = 0; i < hypothesisMatrix.size(); i++) {
    for (int j = 0; j < hypothesisMatrix[0].size(); j++) {
      if (hypothesisMatrix[i][j].second.isMissedDetection) {
        distanceMatrix(i, j) = std::numeric_limits<double>::max();
      } else {
        distanceMatrix(i, j) = hypothesisMatrix[i][j].first;
      }
    }
  }

  HungarianSolver assignmentSolver;
  assignmentSolver.solve(distanceMatrix);
  std::unordered_map<int, int> assignments = assignmentSolver.getAssignments();
  for (const auto& a : assignments) {
    int track = a.first;
    int hyp = a.second;
    associations.insert(std::pair<std::string, Detection>(detectedTracks[track], hypothesisMatrix[track][hyp].second));
  }
  return associations;
}

std::unordered_map<std::string, Tracker::DetectionDict> Tracker::generateTrackHypotheses(std::unordered_map<std::string, Track>& tracks,
                                                                                         const std::vector<Detection>& detections,
                                                                                         double timestamp) const {
  std::unordered_map<std::string, DetectionDict> trackHypotheses;
  Track::Filter::State pred;
  Track::Filter::MeasurementVector predMeas;
  Eigen::Matrix6d cov;
  for (auto& p : tracks) {
    if (timestamp < 0) {
      p.second.predict(detections[0].timestamp);
      pred = p.second.getPrediction();
      predMeas = p.second.predictMeasurement(pred);
      cov = p.second.innovationCovariance();
    } else {
      auto states = p.second.getStates();
      auto it = states.lower_bound(timestamp);
      if (it != states.end()) {
        Track::Filter::State x = it->second;
        double time = it->first;
        p.second.getKF().predict(x, timestamp - time, pred, cov);
        predMeas = p.second.predictMeasurement(pred);
      } else {
        continue;  // did not find associated measurement
      }
    }
    DetectionDict detectionScores;
    Detection missed{};
    detectionScores.insert(std::pair<double, Detection>(missedDistance_, missed));  // insert null hypothesis (no detection for track)
    for (const auto& detection : detections) {
      double dist = mahalanobis(predMeas, detection.measurement(), cov);
      if (dist < missedDistance_) {
        detectionScores.insert(std::pair<double, Detection>(dist, detection));
      }
    }
    trackHypotheses.insert(std::pair<std::string, DetectionDict>(p.first, detectionScores));
  }
  return trackHypotheses;
}

std::vector<std::string> Tracker::deleteTracks(std::unordered_map<std::string, Track>& tracks, double threshold) {
  std::vector<std::string> deletionCandidates;

  for (const auto& t : tracks) {
    double covTrace = t.second.getCovariance().trace();
    if (covTrace > threshold) {
      RCLCPP_DEBUG(rclcpp::get_logger("dynamic_mapping_ros"), 
                   "Deleting track %s: %f > %f", t.first.c_str(), covTrace, threshold);
      deletionCandidates.push_back(t.first);
    }
  }

  for (const std::string& k : deletionCandidates) {
    tracks.erase(k);
  }

  return deletionCandidates;
}

void Tracker::initiateTracks(const std::vector<Detection>& detections) {
  std::vector<Detection> matchedDetections;
  std::vector<Detection> unmatchedDetections;
  if (detections.empty()) {
    return;  // nothing to initiate with
  }
  if (!holdingTracks_.empty()) {
    auto associations = associateDetections(holdingTracks_, detections);
    for (const auto& a : associations) {
      if (!a.second.isMissedDetection) {
        Track::Filter::State postState = holdingTracks_.at(a.first).correct(a.second);
        holdingTracks_.at(a.first).addState(postState, a.second.timestamp);
        matchedDetections.push_back(a.second);
        trackUpdates_[a.first]++;

      } else {
        Track::Filter::State prioState = holdingTracks_.at(a.first).getPrediction();
        holdingTracks_.at(a.first).addState(prioState, detections[0].timestamp);
      }
      if (trackUpdates_[a.first] >= minPoints_) {
        tracks_.emplace(a.first, std::move(holdingTracks_.at(a.first)));
        holdingTracks_.erase(a.first);
        trackUpdates_.erase(a.first);
      }
    }
  }
  auto deletedTracks = deleteTracks(holdingTracks_, initialTraceThreshold_);
  for (auto k : deletedTracks) {
    trackUpdates_.erase(k);
  }
  std::copy_if(detections.begin(), detections.end(), std::back_inserter(unmatchedDetections), [&](const Detection& d) -> bool {
    return (std::find(matchedDetections.begin(), matchedDetections.end(), d) == matchedDetections.end());
  });

  for (const auto& detection : unmatchedDetections) {
    std::string uuid = boost::uuids::to_string(uuidGenerator_());
    Track::FilterParameters params = *filterParams_;
    Track::Filter::State initialState(stateFromDetection(detection), P0_);
    Track t(initialState, params, detection.timestamp, uuid);
    holdingTracks_.emplace(uuid, t);
    trackUpdates_[uuid] = 1;
  }
}
void Tracker::updateTrackLabels(const std::vector<Detection>& labelledDetections) {
  // std::cout << labelledDetections.size() << " detections" << std::endl;
  std::unordered_map<std::string, Detection> associations = associateDetections(tracks_, labelledDetections, true);
  for (const auto& a : associations) {
    if (!a.second.isMissedDetection && a.second.label < 254) {
      tracks_.at(a.first).setLabel(a.second.label);
      // std::cout << "Updated track " << a.first << " to label " << tracks_.at(a.first).getLabel() << std::endl;
    }
  }
}