#include "ground_remover/ground_remover_ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

using namespace ground_remover;

GroundRemoverRos::GroundRemoverRos(const std::string& inputTopic, const std::string& outputTopic,
                                   const std::string& configFilePath, float leaf_size)
    : Node("ground_remover_node") {
  cloudSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      inputTopic, 1, std::bind(&GroundRemoverRos::cloudCallback, this, std::placeholders::_1));
  noGroundCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(outputTopic, 1);
  groundRemover_ = std::make_unique<ElevationMapGroundRemover>(configFilePath, leaf_size);
}

void GroundRemoverRos::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc) {
  PointCloud inCloud;
  PointCloud outCloud;
  // Measure the processing time
  auto start = std::chrono::high_resolution_clock::now();
  pcl::fromROSMsg(*pc, inCloud);
  // Perform ground removal
  groundRemover_->removeGround(inCloud, outCloud);
  // Convert PCL point cloud back to ROS2 message
  sensor_msgs::msg::PointCloud2 outMsg;
  pcl::toROSMsg(outCloud, outMsg);
  outMsg.header = pc->header;
  // Publish the filtered point cloud
  noGroundCloudPublisher_->publish(outMsg);
  auto end = std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(this->get_logger(), "Ground removal: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
}