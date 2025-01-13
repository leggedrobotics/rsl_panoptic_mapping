#ifndef GROUND_REMOVER_ROS_H
#define GROUND_REMOVER_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "elevation_map_ground_remover.hpp"
#include "ground_remover.hpp"

namespace ground_remover {
class GroundRemoverRos{
 public:
  enum ALGO { GRID_MAP = 0, PATCHWORK };
  GroundRemoverRos(rclcpp::Node::SharedPtr node, const std::string& inputTopic, const std::string& outputTopic,
                   const std::string& configFilePath, float leaf_size);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noGroundCloudPublisher_;
  std::unique_ptr<GroundRemover> groundRemover_;
  rclcpp::Node::SharedPtr node_;  // Store the node instance
};
}  // namespace ground_remover

#endif  // GROUND_REMOVER_ROS_H