//
// Created by peyschen on 15/03/23.
//

#include "message_matching/MessageMatcher.h"

void MessageMatcher::getParameters(const ros::NodeHandle& nh) {
  if (!nh.param<std::string>("pointcloud_topic", outParams_.pointCloudTopic_, "pointcloud")) {
    ROS_WARN_STREAM("Did not find 'pointcloud_topic' parameter, defaulting to " << outParams_.pointCloudTopic_);
  }
  if (!nh.param<std::string>("image_topic", outParams_.cameraTopic_, "cameraImage")) {
    ROS_WARN_STREAM("Did not find 'image_topic' parameter, defaulting to " << outParams_.cameraTopic_);
  }
  if (!nh.param<std::string>("output_topic", outParams_.outputTopic_, "matched_messages")) {
    ROS_WARN_STREAM("Did not find 'output_topic' parameter, defaulting to " << outParams_.outputTopic_);
  }
  if (!nh.param<double>("lidar_frequency", outParams_.lidarFrequency_, 10.0)) {
    ROS_WARN_STREAM("Did not find 'lidar_frequency' parameter, defaulting to " << outParams_.lidarFrequency_);
  }
  if (!nh.param<double>("sensor_offset", outParams_.lidarFrequency_, 0.0)) {
    ROS_WARN_STREAM("Did not find 'sensor_offset' parameter, defaulting to " << outParams_.lidarFrequency_);
  }
}

void MessageMatcher::setupRos(ros::NodeHandle& nh) {
  setupMessageFilterChain(nh);

  matchedMessagesPublisher_ = nh.advertise<message_matching_msgs::MatchedPair>(outParams_.outputTopic_, 1, false);
}

void MessageMatcher::setupMessageFilterChain(ros::NodeHandle& nh) {
  pointCloudSubscriber_ = std::make_unique<PointCloudSubscriber>(nh, outParams_.pointCloudTopic_, 1);
  cameraImageSubscriber_ = std::make_unique<CompressedImageSubscriber>(nh, outParams_.cameraTopic_, 1);

  lidarRotationCompensator_ = std::make_unique<LidarRotationCompensator>();

  messageSynchronizer_ = std::make_unique<MessageSynchronizer>();
  output_ = std::make_unique<Output>();

  lidarRotationCompensator_->set_processing_function(
      std::bind(&MessageMatcher::compensateLidarRotation, this, std::placeholders::_1, std::placeholders::_2));
  output_->set_processing_function(std::bind(&MessageMatcher::publishMatchedMessages, this, std::placeholders::_1, std::placeholders::_2));

  lidarRotationCompensator_->connect_to_source(*pointCloudSubscriber_);
  messageSynchronizer_->connect_to_sources(*lidarRotationCompensator_, *cameraImageSubscriber_);
  messageSynchronizer_->connect_to_sink(*output_);
}

void MessageMatcher::setup(ros::NodeHandle& pnh) {
  getParameters(pnh);
  setupRos(pnh);
}
void MessageMatcher::compensateLidarRotation(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr& in,
                                             const LidarRotationCompensator::CallbackFunction& callbackFun) const {
  ros::Time originalTimestamp = in->header.stamp;
  sensor_msgs::PointCloud2 out(*in);
  out.header.stamp = originalTimestamp + ros::Duration().fromSec(outParams_.sensorOffset_ / outParams_.lidarFrequency_);

  callbackFun(boost::make_shared<sensor_msgs::PointCloud2>(out));
}
bool MessageMatcher::publishMatchedMessages(const sensor_msgs::PointCloud2::ConstPtr& matchedPointCloud,
                                            const sensor_msgs::CompressedImage::ConstPtr& matchedCameraImage) {
  message_matching_msgs::MatchedPair msg;
  msg.point_cloud = *matchedPointCloud;
  msg.camera_image = *matchedCameraImage;
  msg.header.stamp = matchedPointCloud->header.stamp;

  matchedMessagesPublisher_.publish(msg);
  return true;
}
