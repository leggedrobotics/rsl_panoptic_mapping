#include <rclcpp/rclcpp.hpp>
#include "ellipsoid_fit/ellipsoid_fit.h"

int main(int argc, char** argv) {
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<rclcpp::Node>("ellipsoid_fit_node");

  // Get parameters from the node
  const std::string input_topic = node->declare_parameter<std::string>("input_topic", "/new_cloud/dynamic");
  const std::string output_topic =  node->declare_parameter<std::string>("output_topic", "/ellipsoid_axes");
  // Create the GroundRemoverRos instance
  auto ellipsoid_fit = std::make_shared<ellipsoid_fit::EllipsoidFit>(
      node, input_topic, output_topic);
  // Spin the node
  rclcpp::spin(node);

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}
