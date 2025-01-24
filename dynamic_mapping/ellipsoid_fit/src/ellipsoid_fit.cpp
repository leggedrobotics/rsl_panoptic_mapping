#include "ellipsoid_fit/ellipsoid_fit.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <fstream>
#include <Eigen/Dense>

namespace ellipsoid_fit {

    EllipsoidFit::EllipsoidFit(
      const rclcpp::Node::SharedPtr& node,
      const std::string& input_topic,
      const std::string& output_topic)
      : node_(node),tfBuffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)) {
    // Initialize the subscriber
    pointCloudSubscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10, std::bind(&EllipsoidFit::callback, this, std::placeholders::_1));

    // Initialize the publisher
    ellipsoidPublisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    listener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_);
  }

void EllipsoidFit::callback(const sensor_msgs::msg::PointCloud2::SharedPtr rock) {
  // Check if the point cloud is empty
  if (rock->height == 0 || rock->width == 0) {
    return;
  }

  // Initialize the TF buffer and listener

  geometry_msgs::msg::TransformStamped lidarToWorld;
  geometry_msgs::msg::TransformStamped worldToCamera;
  geometry_msgs::msg::TransformStamped worldToLidar;
  geometry_msgs::msg::TransformStamped shovelFrame;

  try {
    lidarToWorld = tfBuffer_.lookupTransform("map", "os0_lidar", tf2::TimePointZero);
    worldToCamera = tfBuffer_.lookupTransform("camMainView", "map", tf2::TimePointZero);
    worldToLidar = tfBuffer_.lookupTransform("os0_lidar", "map", tf2::TimePointZero);
    shovelFrame = tfBuffer_.lookupTransform("map", "SHOVEL", tf2::TimePointZero); // TODO: Change to shovel frame
  } catch (tf2::TransformException& e) {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    return;
  }

  sensor_msgs::msg::PointCloud2 rockWorld;
  tf2::doTransform(*rock, rockWorld, lidarToWorld);
  if (!pc_saved) {
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(rockWorld, pc);
    savePointCloudToCSV(pc, "/home/jonas/Coding/moleworks_ext/exts/moleworks_ext/moleworks_ext/ros/scripts/point_cloud.csv");
    
    pc_saved = true;
  }
  // Process the transformed point cloud to fit an ellipsoid
  // (Implementation of ellipsoid fitting goes here)

  // Publish the result
  // ellipsoidPublisher_->publish(rockWorld);
}

void EllipsoidFit::savePointCloudToCSV(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::string& filename) {
  std::ofstream file;
  file.open(filename);

  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open file: %s", filename.c_str());
    return;
  }


  // Write the point cloud data
  for (const auto& point : cloud.points) {
    file << point.x << "," << point.y << "," << point.z << "\n";
  }

  file.close();
  RCLCPP_INFO(node_->get_logger(), "Point cloud saved to: %s", filename.c_str());
}

Eigen::Vector3f EllipsoidFit::calculate_center(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<float> x_values, y_values, z_values;
  for (const auto& point : cloud.points) {
    x_values.push_back(point.x);
    y_values.push_back(point.y);
    z_values.push_back(point.z);
  }

  std::sort(x_values.begin(), x_values.end());
  std::sort(y_values.begin(), y_values.end());
  std::sort(z_values.begin(), z_values.end());

  float center_x = x_values[static_cast<int>(0.9 * x_values.size())]; // TODO : tune
  float center_y = (y_values.front() + y_values.back()) / 2.0f;
  float center_z = (z_values.front() + z_values.back()) / 2.0f;

  return Eigen::Vector3f(center_x, center_y, center_z);
}

void EllipsoidFit::fit_ellipsoide_pca(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  // Convert the point cloud to an Eigen matrix
  Eigen::MatrixXf points(cloud.points.size(), 3);
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    points(i, 0) = cloud.points[i].x;
    points(i, 1) = cloud.points[i].y;
    points(i, 2) = cloud.points[i].z;
  }

  // Calculate the center of the points
  Eigen::Vector3f center = calculate_center(cloud);

  // Center the points around the calculated center
  Eigen::MatrixXf centered_points = points.rowwise() - center.transpose();

  // Calculate the covariance matrix
  Eigen::Matrix3f cov = (centered_points.adjoint() * centered_points) / float(centered_points.rows() - 1);

  // Perform eigen decomposition
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
  Eigen::Vector3f eigenvalues = eig.eigenvalues().real();
  Eigen::Matrix3f eigenvectors = eig.eigenvectors().real();

  // Sort the eigenvalues and eigenvectors
  std::vector<std::pair<float, Eigen::Vector3f>> eigen_pairs;
  for (int i = 0; i < 3; ++i) {
    eigen_pairs.emplace_back(eigenvalues[i], eigenvectors.col(i));
  }
  std::sort(eigen_pairs.begin(), eigen_pairs.end(), [](const auto& a, const auto& b) {
    return a.first > b.first;
  });

  // Extract the sorted eigenvalues and eigenvectors
  for (int i = 0; i < 3; ++i) {
    eigenvalues[i] = eigen_pairs[i].first;
    eigenvectors.col(i) = eigen_pairs[i].second;
  }

  // Calculate the lengths of the ellipsoid axes
  Eigen::Vector3f lengths = 2.0f * eigenvalues.cwiseSqrt();

  // Print the results
  std::cout << "Center: " << center.transpose() << std::endl;
  std::cout << "Eigenvectors: " << std::endl << eigenvectors << std::endl;
  std::cout << "Lengths: " << lengths.transpose() << std::endl;
}

}  // namespace ellipsoid_fit

