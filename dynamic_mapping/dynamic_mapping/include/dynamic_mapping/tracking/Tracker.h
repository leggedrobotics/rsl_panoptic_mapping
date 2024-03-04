#ifndef DYNAMIC_MAPPING_TRACKING_TRACKER_H_
#define DYNAMIC_MAPPING_TRACKING_TRACKER_H_

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <unordered_map>
#include <vector>
#include "dynamic_mapping/tracking/Detection.h"
#include "dynamic_mapping/tracking/Track.h"

class Tracker {
 public:
  explicit Tracker(const std::vector<int>& initiableLabels, const double& missedDistance, const double& covarTraceThreshold,
                   const double& initiationCovarTraceThreshold, const int& minimumMeasurementsForInitiation);
  void track(const std::vector<Detection>& detections);
  void updateTrackLabels(const std::vector<Detection>& labelledDetections);
  std::unordered_map<std::string, Track>* getTracks() { return &tracks_; }

 private:
  using DetectionDict = std::multimap<double, Detection>;

  std::unique_ptr<Track::FilterParameters> filterParams_;

  std::unordered_map<std::string, Track> tracks_;
  std::unordered_map<std::string, Track> holdingTracks_;
  std::unordered_map<std::string, int> trackUpdates_;
  std::unordered_map<std::string, Detection> associateDetections(std::unordered_map<std::string, Track>& tracks,
                                                                 const std::vector<Detection>& detections,
                                                                 bool isLabelUpdate = false) const;
  std::unordered_map<std::string, DetectionDict> generateTrackHypotheses(std::unordered_map<std::string, Track>& tracks,
                                                                         const std::vector<Detection>& detections,
                                                                         double timestamp = -1) const;
  std::vector<std::string> deleteTracks(std::unordered_map<std::string, Track>& tracks, double threshold);
  void initiateTracks(const std::vector<Detection>& detections);
  std::vector<int> initiableLabels_;
  double missedDistance_ = 1.5;
  double covarTraceThreshold_ = 7.5;
  double initialTraceThreshold_ = 35;
  int minPoints_ = 3;
  Track::Filter::StateMatrix P0_;

  boost::uuids::random_generator uuidGenerator_;
};

#endif