// Created by peyschen on 15/03/23

#ifndef DYNAMIC_MAPPING_MESSAGEMATCHER_H
#define DYNAMIC_MAPPING_MESSAGEMATCHER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
//#include <sensor_msgs/msg/image.hpp>
#include <message_matching_msgs/msg/matched_pair.hpp>

// ROS2 message_filters includes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "Parameters.h"

class MessageMatcher
{
public:
  explicit MessageMatcher(const rclcpp::Node::SharedPtr & node);

private:
  /**
   * @brief Declare and retrieve parameters from the ROS2 parameter server
   */
  void getParameters();

  /**
   * @brief Setup subscribers, publishers, and the time synchronizer chain
   */
  void setupRos();

  /**
   * @brief Callback that receives synchronized PointCloud2 + CompressedImage
   */
  void synchronizedCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
                            const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image);

  /**
   * @brief Apply rotation compensation to the incoming point cloud
   */
  sensor_msgs::msg::PointCloud2::ConstSharedPtr
    compensateLidarRotation(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in) const;

  /**
   * @brief Publish the matched messages
   */
  bool publishMatchedMessages(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & matchedPointCloud,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & matchedCameraImage);

private:
  rclcpp::Node::SharedPtr node_;
  Parameters outParams_;

  // Subscribers (message_filters style)
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudOnlySubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr ImageOnlySubscriber_;


  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointCloudSubscriber_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> cameraImageSubscriber_;

  void cameraImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);



  // Synchronizer
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2,
      sensor_msgs::msg::CompressedImage>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publisher
  rclcpp::Publisher<message_matching_msgs::msg::MatchedPair>::SharedPtr matchedMessagesPublisher_;
};

#endif  // DYNAMIC_MAPPING_MESSAGEMATCHER_H
