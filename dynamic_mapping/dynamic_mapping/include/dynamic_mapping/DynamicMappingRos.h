#ifndef DYNAMIC_MAPPING_CPP_DYNAMICMAPPINGROS_H
#define DYNAMIC_MAPPING_CPP_DYNAMICMAPPINGROS_H
#include <dynamic_mapping_msgs/Track.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "dynamic_mapping/MovingObjectsFilter.h"

class DynamicMappingRos {
 public:
  DynamicMappingRos(ros::NodeHandle& nh, const GeneralParameters& generalParameters, const ObjectFilterParameters& objectFilterParameters);
  void callback(sensor_msgs::PointCloud2 raw, sensor_msgs::PointCloud2 noGnd, sensor_msgs::CompressedImage cam);
  void maskCallback(const sensor_msgs::PointCloud2& noGnd, const sensor_msgs::Image& mask);

 private:
  static Eigen::Matrix3Xd rosToEigen(const sensor_msgs::PointCloud2& ros);
  void debugOutput();
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camInfo);
  cv::Mat undistortMask(const cv::Mat& image);

  // ros::NodeHandlePtr nh_;
  cv::Mat cameraIntrinsics_;
  Eigen::Matrix<double, 3, 4> cameraProjection_;
  cv::Mat distorsionCoefficients_;
  cv::Mat map1_, map2_;
  tf2_ros::Buffer tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  ros::Publisher pub_;
  ros::Publisher bboxPub_;
  ros::Publisher trackBoxesPub_;
  ros::Publisher cloudPubStatic_;
  ros::Publisher cloudPubDynamic_;
  ros::Publisher trackPub_;
  ros::Publisher markerPub_;
  ros::Subscriber cameraInfoSubscriber_;
  std::unique_ptr<MovingObjectsFilter> filter_;
  std::mutex camInfoLock_;
  bool hasNewCameraData_ = false;
  sensor_msgs::CameraInfo camInfo_;
  int imageHeight_ = 1152;
  int imageWidth_ = 1376;
  bool verbose_ = false;
  bool shouldUndistort_ = false;

  std::vector<dynamic_mapping_msgs::Track> generateTrackMsgs();
};
#endif  // DYNAMIC_MAPPING_CPP_DYNAMICMAPPINGROS_H
