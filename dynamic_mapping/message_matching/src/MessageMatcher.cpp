#include "message_matching/MessageMatcher.hpp"

// For boost::make_shared in ROS2 code, you can also use std::make_shared.
#include <boost/make_shared.hpp>

MessageMatcher::MessageMatcher(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  // 1. Declare & load parameters
  getParameters();
  // 2. Setup pubs, subs, sync
  setupRos();
}

void MessageMatcher::getParameters()
{
  // Declare parameters with defaults
  node_->declare_parameter<std::string>("pointcloud_topic", "pointcloud");
  node_->declare_parameter<std::string>("image_topic", "cameraImage");
  node_->declare_parameter<std::string>("output_topic", "matched_messages");
  node_->declare_parameter<double>("lidar_frequency", 2.0);
  node_->declare_parameter<double>("sensor_offset", 0.0);

  // Retrieve them into outParams_
  node_->get_parameter("pointcloud_topic", outParams_.pointCloudTopic_);
  node_->get_parameter("image_topic",      outParams_.cameraTopic_);
  node_->get_parameter("output_topic",     outParams_.outputTopic_);
  node_->get_parameter("lidar_frequency",  outParams_.lidarFrequency_);
  node_->get_parameter("sensor_offset",    outParams_.sensorOffset_);

  RCLCPP_WARN_STREAM(node_->get_logger(),
                     "Using parameters:"
                     << " pointcloud_topic=" << outParams_.pointCloudTopic_
                     << ", image_topic=" << outParams_.cameraTopic_
                     << ", output_topic=" << outParams_.outputTopic_
                     << ", lidar_frequency=" << outParams_.lidarFrequency_
                     << ", sensor_offset=" << outParams_.sensorOffset_);
}

void MessageMatcher::setupRos()
{
  // Create publisher
  matchedMessagesPublisher_ = node_->create_publisher<message_matching_msgs::msg::MatchedPair>(
      outParams_.outputTopic_,
      1  /* QoS depth */
  );

  // Create message_filters subscribers
  // The original subscriptions are commented out:
  // pointCloudOnlySubscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     outParams_.pointCloudTopic_,  // Topic name
  //     1,                            // QoS depth
  //     std::bind(&MessageMatcher::pointCloudCallback, this, std::placeholders::_1)  // Callback
  // );
  //
  // ImageOnlySubscriber_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
  //     outParams_.cameraTopic_,  // Topic name
  //     1,                            // QoS depth
  //     std::bind(&MessageMatcher::cameraImageCallback, this, std::placeholders::_1)  // Callback
  // );

  pointCloudSubscriber_ =
    std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        node_,
        outParams_.pointCloudTopic_,
        rclcpp::QoS(20).get_rmw_qos_profile()  // convert QoS to rmw
    );

  cameraImageSubscriber_ =
    std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node_,
        outParams_.cameraTopic_,
        rclcpp::QoS(20).get_rmw_qos_profile()
    );

  // Create synchronizer (now synchronizing a PointCloud2 with an Image)
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(20 /* queue size */),
      *pointCloudSubscriber_,
      *cameraImageSubscriber_);

  // Register synchronized callback
  sync_->registerCallback(
    std::bind(&MessageMatcher::synchronizedCallback, this,
              std::placeholders::_1, std::placeholders::_2));
}

void MessageMatcher::synchronizedCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const sensor_msgs::msg::Image::ConstSharedPtr & image)
{
  //RCLCPP_INFO(node_->get_logger(), "Received synchronized messages!");
  //RCLCPP_INFO(node_->get_logger(), "PointCloud timestamp: %ld.%ld", cloud->header.stamp.sec, cloud->header.stamp.nanosec);
  //RCLCPP_INFO(node_->get_logger(), "Image timestamp: %ld.%ld", image->header.stamp.sec, image->header.stamp.nanosec);
  
  // 1. Compensate the incoming point cloud
  auto compensatedCloud = compensateLidarRotation(cloud);

  // 2. Publish matched messages
  publishMatchedMessages(compensatedCloud, image);
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr
MessageMatcher::compensateLidarRotation(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in) const
{
  rclcpp::Time originalTimestamp = in->header.stamp;
  sensor_msgs::msg::PointCloud2 out(*in);

  // Compute offset duration
  double offset_sec = outParams_.sensorOffset_ / outParams_.lidarFrequency_;
  rclcpp::Duration offset_duration = rclcpp::Duration::from_seconds(offset_sec);
  out.header.stamp = originalTimestamp + offset_duration;

  return std::make_shared<sensor_msgs::msg::PointCloud2>(out);
}

bool MessageMatcher::publishMatchedMessages(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & matchedPointCloud,
  const sensor_msgs::msg::Image::ConstSharedPtr & matchedCameraImage)
{
  // Construct the output message
  message_matching_msgs::msg::MatchedPair msg;
  msg.point_cloud = *matchedPointCloud;
  msg.camera_image = *matchedCameraImage;
  msg.header.stamp = matchedPointCloud->header.stamp;

  matchedMessagesPublisher_->publish(msg);
  return true;
}

void MessageMatcher::cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    //RCLCPP_INFO(node_->get_logger(), "Camera image message received");
}

void MessageMatcher::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
    //RCLCPP_INFO(node_->get_logger(), "Point cloud message received");
}
