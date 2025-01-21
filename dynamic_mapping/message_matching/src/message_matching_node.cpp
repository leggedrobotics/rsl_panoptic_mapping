#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "message_matching/MessageMatcher.hpp"

class MessageMatchingNode {
public:
  MessageMatchingNode() {
    // Create the underlying ROS2 node
    node_ = std::make_shared<rclcpp::Node>("message_matching_node");

    RCLCPP_INFO(node_->get_logger(), "Initializing MessageMatchingNode...");

    // Create and initialize the MessageMatcher
    messageMatcher_ = std::make_unique<MessageMatcher>(node_);

    RCLCPP_INFO(node_->get_logger(), "MessageMatchingNode initialized.");
  }

  void spin() {
    rclcpp::spin(node_);
  }

  ~MessageMatchingNode() {
    RCLCPP_INFO(node_->get_logger(), "Shutting down MessageMatchingNode...");
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<MessageMatcher> messageMatcher_;
};

// Main function to register the node
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Create and manage the MessageMatchingNode
  auto messageMatchingNode = std::make_unique<MessageMatchingNode>();
  messageMatchingNode->spin();

  rclcpp::shutdown();
  return 0;
}
