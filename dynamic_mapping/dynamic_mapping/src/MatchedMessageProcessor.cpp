#include "dynamic_mapping/MatchedMessageProcessor.h"
#include <utility>

using namespace dynamic_mapping;

MatchedMessageProcessor::MatchedMessageProcessor(
    const rclcpp::Node::SharedPtr& node,
    const MessageProcessorParameters& params,
    const std::string& groundRemovalConfig,
    float groundRemovalVoxelSize)
    : node_(node) {
  // Initialize ground remover
  groundRemover_ = std::make_unique<ground_remover::ElevationMapGroundRemover>(groundRemovalConfig, groundRemovalVoxelSize);

  // Publisher for raw camera image
  rawCamImagePublisher_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>("republished_image", 10);

  // Subscriber for matched messages
  matchedMessageSubscriber_ = node_->create_subscription<message_matching_msgs::msg::MatchedPair>(
      params.matchedMessageTopic, 10,
      std::bind(&MatchedMessageProcessor::matchingCallback, this, std::placeholders::_1));

  // Subscriber for segmentation mask
  segMaskSubscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
      params.segmentationMaskTopic, 10,
      std::bind(&MatchedMessageProcessor::segMaskCallback, this, std::placeholders::_1));
}

void MatchedMessageProcessor::matchingCallback(const message_matching_msgs::msg::MatchedPair::SharedPtr matchedMsg) {
  // Remove ground from the point cloud
  sensor_msgs::msg::PointCloud2 groundRemovedCloud = removeGround(matchedMsg->point_cloud);

  if (!waitingForSegmentationMask_) {
    {
      // Lock for thread safety
      std::lock_guard<std::mutex> lck{pointCloudLock_};
      groundRemovedCloud_ = groundRemovedCloud;
      waitingForSegmentationMask_ = true;
    }

    // Publish the raw camera image
    rawCamImagePublisher_->publish(matchedMsg->camera_image);
  }

  // Call the callback function with the matched message data
  callback_(matchedMsg->point_cloud, groundRemovedCloud, matchedMsg->camera_image);
}


sensor_msgs::msg::PointCloud2 MatchedMessageProcessor::removeGround(const sensor_msgs::msg::PointCloud2& inCloud) {
  // Declare input and output point clouds
  ground_remover::PointCloud in;
  ground_remover::PointCloud out;
  sensor_msgs::msg::PointCloud2 outCloud;

  // Convert ROS2 message to PCL format
  pcl::fromROSMsg(inCloud, in);

  // Apply ground removal
  groundRemover_->removeGround(in, out);

  // Convert PCL format back to ROS2 message
  pcl::toROSMsg(out, outCloud);

  // Copy the timestamp from the input cloud to the output cloud
  outCloud.header.stamp = inCloud.header.stamp;
  outCloud.header.frame_id = inCloud.header.frame_id;

  return outCloud;
}

void MatchedMessageProcessor::segMaskCallback(const sensor_msgs::msg::Image::SharedPtr img) {
  if (!waitingForSegmentationMask_) {
    RCLCPP_ERROR(node_->get_logger(), "Received a segmentation mask when not expecting to!");
    return;
  }

  sensor_msgs::msg::PointCloud2 groundRemovedCloud;

  // Ensure thread-safe access to shared resources
  {
    std::lock_guard<std::mutex> lck{pointCloudLock_};
    groundRemovedCloud = groundRemovedCloud_;
    waitingForSegmentationMask_ = false;
  }

  // Call the mask callback with the ground-removed cloud and the segmentation mask
  maskCallback_(groundRemovedCloud, *img);
}


void MatchedMessageProcessor::registerCallbackFunction(
    std::function<void(sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CompressedImage)> callback) {
  callback_ = std::move(callback);
}

void MatchedMessageProcessor::registerMaskCallbackFunction(
    std::function<void(sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image)> maskCallback) {
  maskCallback_ = std::move(maskCallback);
}
