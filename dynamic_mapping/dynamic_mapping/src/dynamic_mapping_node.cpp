#include <ros/console.h>
#include <ros/ros.h>
#include "dynamic_mapping/DynamicMappingRos.h"
#include "dynamic_mapping/MatchedMessageProcessor.h"
#include "dynamic_mapping/Parameters.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_mapping_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  if (ros::console::set_logger_level("ros.grid_map_pcl", ros::console::levels::Warn)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  std::string configFile;
  std::string groundRemovalConfigFile;
  float groundRemovalVoxelSize = 0.15;
  ros::param::get("~config_file", configFile);
  ros::param::get("~ground_removal_config_file", groundRemovalConfigFile);
  ros::param::get("~ground_removal_voxel_size", groundRemovalVoxelSize);

  ROS_INFO_STREAM("Loading config from " << configFile);
  ROS_INFO_STREAM("Loading ground remover config from " << groundRemovalConfigFile);
  ROS_INFO_STREAM("Ground Removal Voxel Size: " << groundRemovalVoxelSize);

  GeneralParameters generalParameters;
  MessageProcessorParameters matcherParameters;
  ObjectFilterParameters filterParameters;
  loadParameters(configFile, &generalParameters);
  loadParameters(configFile, &matcherParameters);
  loadParameters(configFile, &filterParameters);

  DynamicMappingRos dynamicMapper(nh, generalParameters, filterParameters);
  auto callbackFunctor =
      std::bind(&DynamicMappingRos::callback, &dynamicMapper, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  auto maskCallbackFunctor = std::bind(&DynamicMappingRos::maskCallback, &dynamicMapper, std::placeholders::_1, std::placeholders::_2);
  dynamic_mapping::MatchedMessageProcessor matcher(nh, pnh, matcherParameters, groundRemovalConfigFile, groundRemovalVoxelSize);
  matcher.registerCallbackFunction(callbackFunctor);
  matcher.registerMaskCallbackFunction(maskCallbackFunctor);
  ROS_INFO("Started dynamic mapping node");
  ros::spin();
}