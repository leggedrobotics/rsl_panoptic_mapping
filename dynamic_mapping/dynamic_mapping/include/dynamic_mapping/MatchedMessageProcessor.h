#ifndef DYNAMIC_MAPPING_MESSAGE_MATCHER_H_
#define DYNAMIC_MAPPING_MESSAGE_MATCHER_H_
#include <ground_remover/elevation_map_ground_remover.h>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "dynamic_mapping/Parameters.h"
#include "message_matching_msgs/MatchedPair.h"

namespace dynamic_mapping {
class MatchedMessageProcessor {
 public:
  explicit MatchedMessageProcessor(ros::NodeHandle& nh, ros::NodeHandle& pnh, const MessageProcessorParameters& params,
                                   const std::string& groundRemovalConfig, float groundRemovalVoxelSize);
  void registerCallbackFunction(
      std::function<void(sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::CompressedImage)> callback);
  void registerMaskCallbackFunction(std::function<void(sensor_msgs::PointCloud2, sensor_msgs::Image)> maskCallback);

 private:
  void segMaskCallback(const sensor_msgs::Image::ConstPtr& img);
  void matchingCallback(const message_matching_msgs::MatchedPair::ConstPtr& matchedMsg);
  sensor_msgs::PointCloud2 removeGround(const sensor_msgs::PointCloud2& inCloud);

  ros::Publisher rawCamImagePublisher_;
  ros::Subscriber matchedMessageSubscriber_;
  ros::Subscriber segMaskSubscriber_;

  bool waitingForSegmentationMask_ = false;

  sensor_msgs::PointCloud2 groundRemovedCloud_;
  std::mutex pointCloudLock_;

  std::unique_ptr<ground_remover::ElevationMapGroundRemover> groundRemover_;

  std::function<void(sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::CompressedImage)> callback_;
  std::function<void(sensor_msgs::PointCloud2, sensor_msgs::Image)> maskCallback_;
};

}  // namespace dynamic_mapping
#endif