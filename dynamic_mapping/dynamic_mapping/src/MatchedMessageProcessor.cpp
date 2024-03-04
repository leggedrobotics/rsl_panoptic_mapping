#include "dynamic_mapping/MatchedMessageProcessor.h"
#include <utility>

using namespace dynamic_mapping;

MatchedMessageProcessor::MatchedMessageProcessor(ros::NodeHandle& nh, ros::NodeHandle& pnh, const MessageProcessorParameters& params,
                                                 const std::string& groundRemovalConfig, float groundRemovalVoxelSize = 0.15) {
  groundRemover_ = std::make_unique<ground_remover::ElevationMapGroundRemover>(groundRemovalConfig, groundRemovalVoxelSize);
  rawCamImagePublisher_ = pnh.advertise<sensor_msgs::CompressedImage>("republished_image", 1);
  matchedMessageSubscriber_ = pnh.subscribe(params.matchedMessageTopic, 1, &MatchedMessageProcessor::matchingCallback, this);
  segMaskSubscriber_ = pnh.subscribe(params.segmentationMaskTopic, 1, &MatchedMessageProcessor::segMaskCallback, this);
}

void MatchedMessageProcessor::matchingCallback(const message_matching_msgs::MatchedPair::ConstPtr& matchedMsg) {
  sensor_msgs::PointCloud2 groundRemovedCloud = removeGround(matchedMsg->point_cloud);
  if (!waitingForSegmentationMask_) {
    {
      std::lock_guard<std::mutex> lck{pointCloudLock_};
      groundRemovedCloud_ = groundRemovedCloud;
      waitingForSegmentationMask_ = true;
    }

    rawCamImagePublisher_.publish(matchedMsg->camera_image);
  }
  callback_(matchedMsg->point_cloud, groundRemovedCloud, matchedMsg->camera_image);
}

sensor_msgs::PointCloud2 MatchedMessageProcessor::removeGround(const sensor_msgs::PointCloud2& inCloud) {
  ground_remover::PointCloud in;
  ground_remover::PointCloud out;
  sensor_msgs::PointCloud2 outCloud;
  pcl::fromROSMsg(inCloud, in);
  groundRemover_->removeGround(in, out);
  pcl::toROSMsg(out, outCloud);
  outCloud.header.stamp = inCloud.header.stamp;
  return outCloud;
}

void MatchedMessageProcessor::segMaskCallback(const sensor_msgs::Image::ConstPtr& img) {
  if (!waitingForSegmentationMask_) {
    ROS_ERROR("Received a segmentation mask when not expecting to!");
    return;
  }
  sensor_msgs::PointCloud2 groundRemovedCloud;
  {
    std::lock_guard<std::mutex> lck{pointCloudLock_};
    groundRemovedCloud = groundRemovedCloud_;
    waitingForSegmentationMask_ = false;
  }
  maskCallback_(groundRemovedCloud, *img);
}

void MatchedMessageProcessor::registerCallbackFunction(
    std::function<void(sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::CompressedImage)> callback) {
  callback_ = std::move(callback);
}
void MatchedMessageProcessor::registerMaskCallbackFunction(std::function<void(sensor_msgs::PointCloud2, sensor_msgs::Image)> maskCallback) {
  maskCallback_ = std::move(maskCallback);
}