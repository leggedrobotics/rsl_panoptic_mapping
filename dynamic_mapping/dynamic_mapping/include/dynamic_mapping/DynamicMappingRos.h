#ifndef DYNAMIC_MAPPING_CPP_DYNAMICMAPPINGROS_H
#define DYNAMIC_MAPPING_CPP_DYNAMICMAPPINGROS_H

#include <dynamic_mapping_msgs/msg/track.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
//#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Core>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "dynamic_mapping/MovingObjectsFilter.h"
#include <rclcpp/rclcpp.hpp>
#include <jsk_recognition_msgs/msg/bounding_box_array.hpp>
#include <dynamic_mapping_msgs/msg/track_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class DynamicMappingRos {
 public:
  DynamicMappingRos(
      const rclcpp::Node::SharedPtr& node,
      const GeneralParameters& generalParameters,
      const ObjectFilterParameters& objectFilterParameters);

  void callback(sensor_msgs::msg::PointCloud2 raw,
                sensor_msgs::msg::PointCloud2 noGnd,
                sensor_msgs::msg::Image cam);

  void maskCallback(const sensor_msgs::msg::PointCloud2& noGnd,
                    const sensor_msgs::msg::Image& mask);

 private:
  static Eigen::Matrix3Xd rosToEigen(const sensor_msgs::msg::PointCloud2& ros);
  void debugOutput();
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camInfo);
  cv::Mat undistortMask(const cv::Mat& image);

  rclcpp::Node::SharedPtr node_;

  cv::Mat cameraIntrinsics_;
  Eigen::Matrix<double, 3, 4> cameraProjection_;
  cv::Mat distorsionCoefficients_;
  cv::Mat map1_, map2_;
  tf2_ros::Buffer tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<jsk_recognition_msgs::msg::BoundingBoxArray>::SharedPtr bboxPub_;
  rclcpp::Publisher<jsk_recognition_msgs::msg::BoundingBoxArray>::SharedPtr trackBoxesPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPubStatic_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPubDynamic_;
  rclcpp::Publisher<dynamic_mapping_msgs::msg::TrackArray>::SharedPtr trackPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_check_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSubscriber_;

  std::unique_ptr<MovingObjectsFilter> filter_;
  std::mutex camInfoLock_;

  bool hasNewCameraData_ = false;
  sensor_msgs::msg::CameraInfo camInfo_;
  int imageHeight_ = 1152;
  int imageWidth_ = 1376;
  bool verbose_ = true;
  bool shouldUndistort_ = false;

  bool maskCallback_called_ = false;

  std::vector<dynamic_mapping_msgs::msg::Track> generateTrackMsgs();
};

#endif  // DYNAMIC_MAPPING_CPP_DYNAMICMAPPINGROS_H
