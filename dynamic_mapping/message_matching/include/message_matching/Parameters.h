//
// Created by peyschen on 15/03/23.
//

#ifndef DYNAMIC_MAPPING_PARAMETERS_H
#define DYNAMIC_MAPPING_PARAMETERS_H
#include <string>

struct Parameters{
  std::string pointCloudTopic_;
  std::string cameraTopic_;
  std::string outputTopic_;
  double lidarFrequency_;
  double sensorOffset_;
};

#endif  // DYNAMIC_MAPPING_PARAMETERS_H
