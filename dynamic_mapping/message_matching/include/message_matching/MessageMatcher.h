//
// Created by peyschen on 15/03/23.
//

#ifndef DYNAMIC_MAPPING_MESSAGEMATCHER_H
#define DYNAMIC_MAPPING_MESSAGEMATCHER_H

#include <ros/ros.h>
// clang-format off
#include <fkie_message_filters/combiner_policies/approximate_time.h>
#include <fkie_message_filters/combiner.h>
// clang-format on
#include <fkie_message_filters/simple_user_filter.h>
#include <fkie_message_filters/subscriber.h>
#include <fkie_message_filters/user_filter.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_matching_msgs/MatchedPair.h>

#include "Parameters.h"

namespace mf = fkie_message_filters;

class MessageMatcher {
 public:
  void setup(ros::NodeHandle& pnh);

  using PointCloudSubscriber = mf::Subscriber<sensor_msgs::PointCloud2, mf::RosMessageConstPtr>;
  using CompressedImageSubscriber = mf::Subscriber<sensor_msgs::CompressedImage, mf::RosMessageConstPtr>;

  using LidarRotationCompensator = mf::UserFilter<PointCloudSubscriber::Output, PointCloudSubscriber::Output>;

  using MessageSynchronizer =
      mf::Combiner<mf::combiner_policies::ApproximateTime, PointCloudSubscriber::Output, CompressedImageSubscriber::Output>;
  using Output = mf::SimpleUserFilter<MessageSynchronizer::Output>;

 private:
  void getParameters(const ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);

  void setupMessageFilterChain(ros::NodeHandle& nh);

  void compensateLidarRotation(const sensor_msgs::PointCloud2::ConstPtr& in, const LidarRotationCompensator::CallbackFunction& callbackFun) const;
  bool publishMatchedMessages(const sensor_msgs::PointCloud2::ConstPtr& matchedPointCloud,
                              const sensor_msgs::CompressedImage::ConstPtr& matchedCameraImage);

  Parameters outParams_;

  std::unique_ptr<PointCloudSubscriber> pointCloudSubscriber_;
  std::unique_ptr<CompressedImageSubscriber> cameraImageSubscriber_;

  std::unique_ptr<LidarRotationCompensator> lidarRotationCompensator_;
  std::unique_ptr<MessageSynchronizer> messageSynchronizer_;
  std::unique_ptr<Output> output_;

  ros::Publisher matchedMessagesPublisher_;
};

#endif  // DYNAMIC_MAPPING_MESSAGEMATCHER_H
