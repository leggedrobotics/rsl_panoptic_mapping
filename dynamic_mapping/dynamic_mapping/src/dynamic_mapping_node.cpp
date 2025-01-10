#include <rclcpp/rclcpp.hpp>
#include "dynamic_mapping/DynamicMappingRos.h"
#include "dynamic_mapping/MatchedMessageProcessor.h"
#include "dynamic_mapping/Parameters.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("dynamic_mapping_node");

  // Logger configuration (no direct equivalent for console level setting in ROS2)
  RCLCPP_INFO(node->get_logger(), "Starting dynamic mapping node");

  std::string configFile;
  std::string groundRemovalConfigFile;
  float groundRemovalVoxelSize = 0.15;

  // Parameter retrieval
  node->declare_parameter<std::string>("config_file", "");
  node->declare_parameter<std::string>("ground_removal_config_file", "");
  node->declare_parameter<float>("ground_removal_voxel_size", 0.15);

  node->get_parameter("config_file", configFile);
  node->get_parameter("ground_removal_config_file", groundRemovalConfigFile);
  node->get_parameter("ground_removal_voxel_size", groundRemovalVoxelSize);

  RCLCPP_INFO(node->get_logger(), "Loading config from %s", configFile.c_str());
  RCLCPP_INFO(node->get_logger(), "Loading ground remover config from %s", groundRemovalConfigFile.c_str());
  RCLCPP_INFO(node->get_logger(), "Ground Removal Voxel Size: %f", groundRemovalVoxelSize);

  GeneralParameters generalParameters;
  MessageProcessorParameters matcherParameters;
  ObjectFilterParameters filterParameters;
  loadParameters(configFile, &generalParameters);
  loadParameters(configFile, &matcherParameters);
  loadParameters(configFile, &filterParameters);

  auto dynamicMapper = std::make_shared<DynamicMappingRos>(node, generalParameters, filterParameters);
  
  auto callbackFunctor = std::bind(&DynamicMappingRos::callback, dynamicMapper, 
                                   std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  auto maskCallbackFunctor = std::bind(&DynamicMappingRos::maskCallback, dynamicMapper, 
                                       std::placeholders::_1, std::placeholders::_2);

  auto matcher = std::make_shared<dynamic_mapping::MatchedMessageProcessor>(node, matcherParameters, 
                                                                            groundRemovalConfigFile, groundRemovalVoxelSize);
  matcher->registerCallbackFunction(callbackFunctor);
  matcher->registerMaskCallbackFunction(maskCallbackFunctor);

  RCLCPP_INFO(node->get_logger(), "Started dynamic mapping node");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
