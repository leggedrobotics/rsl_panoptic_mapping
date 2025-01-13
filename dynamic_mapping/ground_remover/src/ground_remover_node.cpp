#include <rclcpp/rclcpp.hpp>
#include "ground_remover/ground_remover_ros.hpp"

int main(int argc, char** argv) {
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<rclcpp::Node>("ground_remover_node");

  // Get parameters from the node
  const std::string configFilePath = node->declare_parameter<std::string>("config_filepath", "");
  const std::string input_topic = node->declare_parameter<std::string>("input_topic", "/ouster_points_self_filtered");
  const std::string output_topic = "no_ground_cloud";
  const float leaf_size = node->declare_parameter<float>("voxel_size", 0.15);

  // Create the GroundRemoverRos instance
  auto groundRemover = std::make_shared<ground_remover::GroundRemoverRos>(
      node, input_topic, output_topic, configFilePath, leaf_size);

  // Spin the node
  rclcpp::spin(node);

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}
