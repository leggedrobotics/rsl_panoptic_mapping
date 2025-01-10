#ifndef GROUND_REMOVER_ROS_H
#define GROUND_REMOVER_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "elevation_map_ground_remover.h"
#include "ground_remover.h"

namespace ground_remover {
class GroundRemoverRos : public rclcpp::Node {
 public:
  enum ALGO { GRID_MAP = 0, PATCHWORK };
  GroundRemoverRos(const std::string& inputTopic, const std::string& outputTopic,
                   const std::string& configFilePath, float leaf_size);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noGroundCloudPublisher_;
  std::unique_ptr<GroundRemover> groundRemover_;
};
}  // namespace ground_remover

#endif  // GROUND_REMOVER_ROS_H