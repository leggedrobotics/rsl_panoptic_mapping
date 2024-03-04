#ifndef DYNAMIC_MAPPING_CPP_PARAMETERS_H
#define DYNAMIC_MAPPING_CPP_PARAMETERS_H
#include <string>
#include <utility>
#include <vector>

struct ObjectFilterParameters {
  std::vector<int> labels;
  double missedDetectionDistance;
  double deletionCovarThreshold;
  double initiationCovarThreshold;
  int minimumInitiationPoints;
  int minimumClusterSize;
  double velocityFilterThreshold;
  double boundingBoxMargin;
  double extentVelocityThreshold;
  double shovelDistanceThreshold;
  bool strictSegmentation;
  bool segmentOnGroundRemovedCloud;
};

struct MessageProcessorParameters {
  std::string matchedMessageTopic;
  std::string segmentationMaskTopic;
};

struct GeneralParameters {
  bool verbose;
  std::string clusterTopic;
  std::string outputTopic;
  std::string detectionTopic;
  std::string trackTopic;
  std::string cameraInfoTopic;
  bool shouldUndistort;
};

#endif  // DYNAMIC_MAPPING_CPP_PARAMETERS_H
