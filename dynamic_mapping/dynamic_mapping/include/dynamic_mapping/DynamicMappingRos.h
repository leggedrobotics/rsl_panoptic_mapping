#ifndef DYNAMIC_MAPPING_CPP_DYNAMICMAPPINGROS_H
#define DYNAMIC_MAPPING_CPP_DYNAMICMAPPINGROS_H

#include <dynamic_mapping_msgs/msg/track.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Core>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "dynamic_mapping/moving_objects_filter.hpp"

class DynamicMappingRos : public rclcpp::Node {
 public:
  DynamicMappingRos(const GeneralParameters& generalParameters,
                    const ObjectFilterParameters& objectFilterParameters);
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr raw,
                const sensor_msgs::msg::PointCloud2::SharedPtr noGnd,
                const sensor_msgs::msg::CompressedImage::SharedPtr cam);
  void maskCallback(const sensor_msgs::msg::PointCloud2::SharedPtr noGnd,
                    const sensor_msgs::msg::Image::SharedPtr mask);

 private:
  static Eigen::Matrix3Xd rosToEigen(const sensor_msgs::msg::PointCloud2& ros);
  void debugOutput();
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camInfo);
  cv::Mat undistortMask(const cv::Mat& image);

  cv::Mat cameraIntrinsics_;
  Eigen::Matrix<double, 3, 4> cameraProjection_;
  cv::Mat distorsionCoefficients_;
  cv::Mat map1_, map2_;
  tf2_ros::Buffer tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPubStatic_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPubDynamic_;
  rclcpp::Publisher<dynamic_mapping_msgs::msg::Track>::SharedPtr trackPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSubscriber_;
  std::unique_ptr<MovingObjectsFilter> filter_;
  std::mutex camInfoLock_;
  bool hasNewCameraData_ = false;
  sensor_msgs::msg::CameraInfo camInfo_;
  int imageHeight_ = 1152;
  int imageWidth_ = 1376;
  bool verbose_ = false;
  bool shouldUndistort_ = false;

  std::vector<dynamic_mapping_msgs::msg::Track> generateTrackMsgs();
};

#endif  // DYNAMIC_MAPPING_CPP_DYNAMICMAPPINGROS_H
