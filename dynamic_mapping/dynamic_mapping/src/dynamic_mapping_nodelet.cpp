#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/ros.h>
#include "dynamic_mapping/DynamicMappingRos.h"
#include "dynamic_mapping/MatchedMessageProcessor.h"
#include "dynamic_mapping/Parameters.h"

class DynamicMappingNodelet : public nodelet::Nodelet {
 private:
  void onInit() override {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();
    if (ros::console::set_logger_level("ros.grid_map_pcl", ros::console::levels::Warn)) {
      ros::console::notifyLoggerLevelsChanged();
    }
    std::string configFile;
    std::string groundRemovalConfigFile;
    float groundRemovalVoxelSize = 0.15;
    pnh.getParam("config_file", configFile);
    pnh.getParam("ground_removal_config_file", groundRemovalConfigFile);
    pnh.getParam("ground_removal_voxel_size", groundRemovalVoxelSize);

    ROS_INFO_STREAM("Loading config from " << configFile);
    ROS_INFO_STREAM("Loading ground remover config from " << groundRemovalConfigFile);
    ROS_INFO_STREAM("Ground Removal Voxel Size: " << groundRemovalVoxelSize);

    GeneralParameters generalParameters;
    MessageProcessorParameters matcherParameters;
    ObjectFilterParameters filterParameters;
    loadParameters(configFile, &generalParameters);
    loadParameters(configFile, &matcherParameters);
    loadParameters(configFile, &filterParameters);

    dynamicMapper_ = std::make_unique<DynamicMappingRos>(nh, generalParameters, filterParameters);
    auto callbackFunctor = [&](sensor_msgs::PointCloud2 pc1, sensor_msgs::PointCloud2 pc2, sensor_msgs::CompressedImage cimg) {
      dynamicMapper_->callback(pc1, pc2, cimg);
    };
    auto maskCallbackFunctor = [&](sensor_msgs::PointCloud2 pc1, sensor_msgs::Image img) { dynamicMapper_->maskCallback(pc1, img); };
    matcher_ = std::make_unique<dynamic_mapping::MatchedMessageProcessor>(nh, pnh, matcherParameters, groundRemovalConfigFile,
                                                                          groundRemovalVoxelSize);
    matcher_->registerCallbackFunction(callbackFunctor);
    matcher_->registerMaskCallbackFunction(maskCallbackFunctor);
    ROS_INFO("Started dynamic mapping node");
  }

  std::unique_ptr<DynamicMappingRos> dynamicMapper_;
  std::unique_ptr<dynamic_mapping::MatchedMessageProcessor> matcher_;
};

PLUGINLIB_EXPORT_CLASS(DynamicMappingNodelet, nodelet::Nodelet);