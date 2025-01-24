#ifndef ELLIPSOID_FIT_H
#define ELLIPSOID_FIT_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
namespace ellipsoid_fit {

class EllipsoidFit {
public:

  EllipsoidFit(
        const rclcpp::Node::SharedPtr& node,
        const std::string& input_topic,
        const std::string& output_topic);
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr rock);

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tfBuffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ellipsoidPublisher_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;

  void savePointCloudToCSV(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::string& filename);

  bool pc_saved = false;
};

}  // namespace ground_remover

#endif  // ELLIPSOID_FIT_H