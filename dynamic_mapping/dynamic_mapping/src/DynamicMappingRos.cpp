#include "dynamic_mapping/dynamic_mapping_ros.h"
#include <cv_bridge/cv_bridge.h>
#include <dynamic_mapping_msgs/msg/track_array.hpp>
#include <jsk_recognition_msgs/msg/bounding_box_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/msg/marker_array.hpp>

size_t Detection::id_ = 0;

DynamicMappingRos::DynamicMappingRos(const GeneralParameters& generalParameters,
                                     const ObjectFilterParameters& objectFilterParameters)
    : Node("dynamic_mapping_ros"), verbose_(generalParameters.verbose) {
  listener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_, this);
  if (verbose_) {
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(generalParameters.clusterTopic, 1);
    bboxPub_ = this->create_publisher<jsk_recognition_msgs::msg::BoundingBoxArray>(generalParameters.detectionTopic, 1);
    trackBoxesPub_ = this->create_publisher<jsk_recognition_msgs::msg::BoundingBoxArray>(generalParameters.trackTopic, 1);
  }
  cloudPubStatic_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(generalParameters.outputTopic + "/static", 1);
  cloudPubDynamic_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(generalParameters.outputTopic + "/dynamic", 1);
  trackPub_ = this->create_publisher<dynamic_mapping_msgs::msg::TrackArray>("/tracks", 1);
  markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers", 1);

  // clang-format off
  cameraIntrinsics_ = (cv::Mat_<double>(3, 3) <<   767.239452,        0.0, 814.414996,
                                                          0.0, 767.168172, 597.978256,
                                                          0.0,        0.0,        1.0);
  cameraProjection_ <<   739.813574,        0.0, 821.512688, 0.0,
                                0.0, 748.554394, 598.101226, 0.0,
                                0.0,        0.0,        1.0, 0.0;
  // clang-format on
  distorsionCoefficients_ = (cv::Mat_<double>(1, 5) << -0.044303, 0.006917, -0.000472, -0.000009, 0);
  cv::initUndistortRectifyMap(cameraIntrinsics_, distorsionCoefficients_, cv::Mat_<double>::eye(3, 3), cameraIntrinsics_,
                              cv::Size(imageWidth_, imageHeight_), CV_16SC2, map1_, map2_);

  filter_ = std::make_unique<MovingObjectsFilter>(objectFilterParameters);
  filter_->updateCameraProjection(cameraProjection_);
  cameraInfoSubscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      generalParameters.cameraInfoTopic, 1, std::bind(&DynamicMappingRos::cameraInfoCallback, this, std::placeholders::_1));
}

void DynamicMappingRos::callback(const sensor_msgs::msg::PointCloud2::SharedPtr raw,
                                 const sensor_msgs::msg::PointCloud2::SharedPtr noGnd,
                                 const sensor_msgs::msg::CompressedImage::SharedPtr cam) {
  auto start = std::chrono::high_resolution_clock::now();
  {
    std::lock_guard<std::mutex> guard{camInfoLock_};
    if (hasNewCameraData_) {
      filter_->updateCameraProjection(cameraProjection_);
      cv::initUndistortRectifyMap(cameraIntrinsics_, distorsionCoefficients_, cv::Mat_<double>::eye(3, 3), cameraIntrinsics_,
                                  cv::Size(imageWidth_, imageHeight_), CV_16SC2, map1_, map2_);

      hasNewCameraData_ = false;
    }
  }

  geometry_msgs::msg::TransformStamped lidarToWorld, worldToCamera, worldToLidar, shovelFrame;
  try {
    lidarToWorld = tfBuffer_.lookupTransform("map", "os_sensor", tf2::TimePointZero);
    worldToCamera = tfBuffer_.lookupTransform("camMainView", "map", tf2::TimePointZero);
    worldToLidar = tfBuffer_.lookupTransform("os_sensor", "map", tf2::TimePointZero);
    shovelFrame = tfBuffer_.lookupTransform("map", "SHOVEL", tf2::TimePointZero);
  } catch (tf2::LookupException& e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return;
  }

  // Rest of the callback implementation

  sensor_msgs::msg::PointCloud2 rawWorld, noGndWorld;
  tf2::doTransform(raw, rawWorld, lidarToWorld);
  tf2::doTransform(noGnd, noGndWorld, lidarToWorld);

  Eigen::Matrix3Xd fullCloud = rosToEigen(rawWorld);
  Eigen::Matrix3Xd noGroundCloud = rosToEigen(noGndWorld);

  filter_->setCameraTransform(tf2::transformToEigen(worldToCamera));
  filter_->filterObjects(fullCloud, noGroundCloud, node_->now().seconds());
  filter_->setShovelFrame(tf2::transformToEigen(shovelFrame));

  sensor_msgs::msg::PointCloud2 outputStaticSensorFrame;
  sensor_msgs::msg::PointCloud2 outputDynamicSensorFrame;

  tf2::doTransform(eigenToRos(filter_->getFilteredCloudStatic(), "map"), outputStaticSensorFrame, worldToLidar);
  tf2::doTransform(eigenToRos(filter_->getFilteredCloudDynamic(), "map"), outputDynamicSensorFrame, worldToLidar);

  cloudPubStatic_->publish(outputStaticSensorFrame);
  cloudPubDynamic_->publish(outputDynamicSensorFrame);
  dynamic_mapping_msgs::msg::TrackArray tracks;
  tracks.tracks = generateTrackMsgs();
  tracks.header.frame_id = "map";
  tracks.header.stamp = raw.header.stamp;

  if (!tracks.tracks.empty()) {
    trackPub_->publish(tracks);
  }

  if (verbose_) {
    debugOutput();
  }

  auto duration = std::chrono::high_resolution_clock::now() - start;
  RCLCPP_INFO(this->get_logger(), "Pipeline took: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());

}

Eigen::Matrix3Xd DynamicMappingRos::rosToEigen(const sensor_msgs::msg::PointCloud2& ros) {
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pcl::fromROSMsg(ros, pclCloud);

  // pcl::PointCloud's getMatrixXfMap() function returns a map of the data.
  return pclCloud.getMatrixXfMap().cast<double>();
}

void DynamicMappingRos::debugOutput() {
  jsk_recognition_msgs::msg::BoundingBoxArray bboxArr;
  std::vector<jsk_recognition_msgs::msg::BoundingBox> trackBoxes;
  std::vector<jsk_recognition_msgs::msg::BoundingBox> detectionBoxes;
  visualization_msgs::msg::MarkerArray markerArray;

  // Process detection boxes
  for (const auto& box : filter_->getDetectionBoxes()) {
    detectionBoxes.push_back(generateBoundingBoxMsg(box));
  }

  // Process track boxes
  for (const auto& box : filter_->getTrackBoxes()) {
    trackBoxes.push_back(generateBoundingBoxMsg(box));
    markerArray.markers.push_back(createMarker(box));
  }

  // Publish detection boxes
  bboxArr.header.frame_id = "map";
  bboxArr.header.stamp = node_->now();
  bboxArr.boxes = detectionBoxes;
  bboxPub_->publish(bboxArr);

  // Publish track boxes
  bboxArr.boxes = trackBoxes;
  trackBoxesPub_->publish(bboxArr);

  // Publish markers
  markerPub_->publish(markerArray);
}

cv::Mat DynamicMappingRos::undistortMask(const cv::Mat& image) {
  cv::Mat undistorted;
  {
    std::lock_guard<std::mutex> guard{camInfoLock_};
    cv::remap(image, undistorted, map1_, map2_, cv::INTER_LINEAR);
  }
  return undistorted;
}

void DynamicMappingRos::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camInfo) {
  {
    std::lock_guard<std::mutex> lock{camInfoLock_};
    if (!((camInfo->d == camInfo_.d) && (camInfo->k == camInfo_.k) && (camInfo->p == camInfo_.p))) {
      imageHeight_ = camInfo->height;
      imageWidth_ = camInfo->width;
      auto distCoeffs = camInfo->d;
      auto intrinsics = camInfo->k;
      auto projection = camInfo->p;

      auto newIntrinsics = cv::Mat(3, 3, CV_64F, intrinsics.data());
      auto newProjection = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(projection.data());

      cameraIntrinsics_ = newIntrinsics.clone();
      cameraProjection_ = newProjection;

      cv::Mat newCoeffs;
      if (!distCoeffs.empty()) {
        newCoeffs = cv::Mat(1, 5, CV_64F, distCoeffs.data());
      } else {
        newCoeffs = cv::Mat::zeros(1, 5, CV_64F);
      };
      distorsionCoefficients_ = newCoeffs.clone();

      camInfo_ = *camInfo;
      hasNewCameraData_ = true;
    }
  }
}

void DynamicMappingRos::maskCallback(const sensor_msgs::msg::PointCloud2& noGnd, const sensor_msgs::msg::Image& mask) {
  geometry_msgs::msg::TransformStamped lidarToWorld, worldToCamera;

  try {
    // Retrieve transforms
    lidarToWorld = tfBuffer_.lookupTransform("map", "os_sensor", tf2::TimePointZero);
    worldToCamera = tfBuffer_.lookupTransform("camMainView", "map", tf2::TimePointZero);
  } catch (const tf2::TransformException& e) {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    return;
  }

  // Transform the input point cloud to world frame
  sensor_msgs::msg::PointCloud2 noGndWorld;
  tf2::doTransform(noGnd, noGndWorld, lidarToWorld);
  Eigen::Matrix3Xd noGroundCloud = rosToEigen(noGndWorld);

  // Convert the ROS Image message to OpenCV format
  cv::Mat image;
  cv::Mat(mask.height, mask.width, CV_8UC3, const_cast<uchar*>(&mask.data[0]), mask.step).copyTo(image);

  if (mask.encoding == "rgb8") {
    // Manually swap red and blue channels for BGR conversion
    cv::Mat rgb[3], tmp;
    cv::split(image, rgb);
    tmp = rgb[0];
    rgb[0] = rgb[2];
    rgb[2] = tmp;
    cv::merge(rgb, 3, image);
  }

  // Optionally undistort the mask
  cv::Mat segmentationMask = shouldUndistort_ ? undistortMask(image) : image;

  // Update the filter with the camera transform and segmentation mask
  filter_->setCameraTransform(tf2::transformToEigen(worldToCamera));
  filter_->updateLabels(noGroundCloud, segmentationMask, rclcpp::Time(noGnd.header.stamp).seconds());

  // Publish labeled clusters if verbose mode is enabled
  if (verbose_) {
    pub_->publish(generateClusterPointCloud(filter_->getLabelledClusters()));
  }
}


std::vector<dynamic_mapping_msgs::msg::Track> DynamicMappingRos::generateTrackMsgs() {
  std::vector<dynamic_mapping_msgs::msg::Track> tracks;
  auto filterTracks = filter_->getTracks();

  for (const auto& track : filterTracks) {
    if (track.getLabel() < 0) continue;

    dynamic_mapping_msgs::msg::Track trackMsg;
    auto trackState = track.getState();

    // Set position
    trackMsg.pose.position.x = trackState.state[0];
    trackMsg.pose.position.y = trackState.state[2];
    trackMsg.pose.position.z = trackState.state[4];

    // Set orientation (default to identity quaternion)
    trackMsg.pose.orientation.w = 1.0;

    // Set velocity
    trackMsg.velocity.linear.x = trackState.state[1];
    trackMsg.velocity.linear.y = trackState.state[3];
    trackMsg.velocity.linear.z = trackState.state[5];

    // Set dimensions
    trackMsg.dimensions.x = trackState.state[6];
    trackMsg.dimensions.y = trackState.state[7];
    trackMsg.dimensions.z = trackState.state[8];

    // Set label and UUID
    trackMsg.label = track.getLabel();
    trackMsg.uuid = track.getUuid();

    tracks.push_back(trackMsg);
  }

  return tracks;
}
