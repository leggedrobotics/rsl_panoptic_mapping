// MatchedMessageProcessor.hpp
#ifndef DYNAMIC_MAPPING_MESSAGE_MATCHER_H_
#define DYNAMIC_MAPPING_MESSAGE_MATCHER_H_

#include <ground_remover/elevation_map_ground_remover.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "dynamic_mapping/Parameters.h"
#include "message_matching_msgs/msg/matched_pair.hpp"

#include <mutex>
#include <functional>
#include <memory>

namespace dynamic_mapping {

class MatchedMessageProcessor {
public:
  explicit MatchedMessageProcessor(
      const rclcpp::Node::SharedPtr& node,
      const MessageProcessorParameters& params,
      const std::string& groundRemovalConfig,
      float groundRemovalVoxelSize);

  void registerCallbackFunction(
      std::function<void(sensor_msgs::msg::PointCloud2,
                         sensor_msgs::msg::PointCloud2,
                         sensor_msgs::msg::Image)> callback);

  void registerMaskCallbackFunction(
      std::function<void(sensor_msgs::msg::PointCloud2,
                         sensor_msgs::msg::Image)> maskCallback);

private:
  void segMaskCallback(const sensor_msgs::msg::Image::SharedPtr img);
  void matchingCallback(const message_matching_msgs::msg::MatchedPair::SharedPtr matchedMsg);
  sensor_msgs::msg::PointCloud2 removeGround(const sensor_msgs::msg::PointCloud2& inCloud);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rawCamImagePublisher_;
  rclcpp::Subscription<message_matching_msgs::msg::MatchedPair>::SharedPtr matchedMessageSubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr segMaskSubscriber_;

  bool waitingForSegmentationMask_ = false;

  sensor_msgs::msg::PointCloud2 groundRemovedCloud_;
  std::mutex pointCloudLock_;

  std::unique_ptr<ground_remover::ElevationMapGroundRemover> groundRemover_;

  std::function<void(sensor_msgs::msg::PointCloud2,
                     sensor_msgs::msg::PointCloud2,
                     sensor_msgs::msg::Image)>
      callback_;

  std::function<void(sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image)> maskCallback_;
};

}  // namespace dynamic_mapping

#endif