#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "dynamic_mapping/DynamicMappingRos.h"
#include "dynamic_mapping/MatchedMessageProcessor.h"
#include "dynamic_mapping/Parameters.h"

class DynamicMappingNode : public rclcpp::Node {
 public:
  explicit DynamicMappingNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("dynamic_mapping_node", options) {
    RCLCPP_INFO(this->get_logger(), "Initializing DynamicMappingNode...");

    // Declare parameters
    this->declare_parameter<std::string>("config_file", "");
    this->declare_parameter<std::string>("ground_removal_config_file", "");
    this->declare_parameter<float>("ground_removal_voxel_size", 0.15);

    // Get parameters
    std::string configFile;
    std::string groundRemovalConfigFile;
    float groundRemovalVoxelSize = 0.15;

    this->get_parameter("config_file", configFile);
    this->get_parameter("ground_removal_config_file", groundRemovalConfigFile);
    this->get_parameter("ground_removal_voxel_size", groundRemovalVoxelSize);

    RCLCPP_INFO(this->get_logger(), "Loading config from %s", configFile.c_str());
    RCLCPP_INFO(this->get_logger(), "Loading ground remover config from %s", groundRemovalConfigFile.c_str());
    RCLCPP_INFO(this->get_logger(), "Ground Removal Voxel Size: %f", groundRemovalVoxelSize);

    // Load parameters
    GeneralParameters generalParameters;
    MessageProcessorParameters matcherParameters;
    ObjectFilterParameters filterParameters;
    loadParameters(configFile, &generalParameters);
    loadParameters(configFile, &matcherParameters);
    loadParameters(configFile, &filterParameters);

    // Initialize dynamic mapper
    dynamicMapper_ = std::make_shared<DynamicMappingRos>(*this, generalParameters, filterParameters);

    // Create callback functors
    auto callbackFunctor = [this](sensor_msgs::msg::PointCloud2 pc1, 
                                   sensor_msgs::msg::PointCloud2 pc2, 
                                   sensor_msgs::msg::CompressedImage cimg) {
      dynamicMapper_->callback(pc1, pc2, cimg);
    };

    auto maskCallbackFunctor = [this](sensor_msgs::msg::PointCloud2 pc1, 
                                       sensor_msgs::msg::Image img) {
      dynamicMapper_->maskCallback(pc1, img);
    };

    // Initialize message matcher
    matcher_ = std::make_shared<dynamic_mapping::MatchedMessageProcessor>(*this, matcherParameters, 
                                                                          groundRemovalConfigFile, groundRemovalVoxelSize);
    matcher_->registerCallbackFunction(callbackFunctor);
    matcher_->registerMaskCallbackFunction(maskCallbackFunctor);

    RCLCPP_INFO(this->get_logger(), "Dynamic mapping node initialized successfully.");
  }

 private:
  std::shared_ptr<DynamicMappingRos> dynamicMapper_;
  std::shared_ptr<dynamic_mapping::MatchedMessageProcessor> matcher_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(DynamicMappingNode)
