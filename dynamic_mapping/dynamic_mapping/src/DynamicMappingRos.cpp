#include "dynamic_mapping/DynamicMappingRos.h"
#include <cv_bridge/cv_bridge.h>
#include <dynamic_mapping_msgs/TrackArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/MarkerArray.h>

size_t Detection::id_ = 0;
DynamicMappingRos::DynamicMappingRos(ros::NodeHandle& nh, const GeneralParameters& generalParameters,
                                     const ObjectFilterParameters& objectFilterParameters)
    : verbose_(generalParameters.verbose) {
  listener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_);
  if (verbose_) {
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(generalParameters.clusterTopic, 1);
    bboxPub_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(generalParameters.detectionTopic, 1);
    trackBoxesPub_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(generalParameters.trackTopic, 1);
  }
  cloudPubStatic_ = nh.advertise<sensor_msgs::PointCloud2>(generalParameters.outputTopic + "/static", 1);
  cloudPubDynamic_ = nh.advertise<sensor_msgs::PointCloud2>(generalParameters.outputTopic + "/dynamic", 1);
  trackPub_ = nh.advertise<dynamic_mapping_msgs::TrackArray>("/tracks", 1);
  markerPub_ = nh.advertise<visualization_msgs::MarkerArray>("/markers", 1);

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
  cameraInfoSubscriber_ = nh.subscribe(generalParameters.cameraInfoTopic, 1, &DynamicMappingRos::cameraInfoCallback, this);
}

void DynamicMappingRos::callback(sensor_msgs::PointCloud2 raw, sensor_msgs::PointCloud2 noGnd, sensor_msgs::CompressedImage cam) {
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

  geometry_msgs::TransformStamped lidarToWorld, worldToCamera, worldToLidar, shovelFrame;
  try {
    lidarToWorld = tfBuffer_.lookupTransform("map", "os_sensor", ros::Time(0));
    worldToCamera = tfBuffer_.lookupTransform("camMainView", "map", ros::Time(0));
    worldToLidar = tfBuffer_.lookupTransform("os_sensor", "map", ros::Time(0));
    shovelFrame = tfBuffer_.lookupTransform("map", "SHOVEL", ros::Time(0));
  } catch (tf2::LookupException& e) {
    ROS_ERROR("%s", e.what());
    return;
  }
  sensor_msgs::PointCloud2 rawWorld, noGndWorld;
  tf2::doTransform(raw, rawWorld, lidarToWorld);
  tf2::doTransform(noGnd, noGndWorld, lidarToWorld);
  Eigen::Matrix3Xd fullCloud = rosToEigen(rawWorld);
  Eigen::Matrix3Xd noGroundCloud = rosToEigen(noGndWorld);

  filter_->setCameraTransform(tf2::transformToEigen(worldToCamera));
  filter_->filterObjects(fullCloud, noGroundCloud, ros::Time::now().toSec());
  filter_->setShovelFrame(tf2::transformToEigen(shovelFrame));
  sensor_msgs::PointCloud2 outputStaticSensorFrame;
  sensor_msgs::PointCloud2 outputDynamicSensorFrame;
  tf2::doTransform(eigenToRos(filter_->getFilteredCloudStatic(), "map"), outputStaticSensorFrame, worldToLidar);
  tf2::doTransform(eigenToRos(filter_->getFilteredCloudDynamic(), "map"), outputDynamicSensorFrame, worldToLidar);
  cloudPubStatic_.publish(outputStaticSensorFrame);
  cloudPubDynamic_.publish(outputDynamicSensorFrame);

  dynamic_mapping_msgs::TrackArray tracks;
  tracks.tracks = generateTrackMsgs();
  tracks.header.frame_id = "map";
  tracks.header.stamp = raw.header.stamp;

  if (!tracks.tracks.empty()) {
    trackPub_.publish(tracks);
  }

  if (verbose_) {
    debugOutput();
  }
  auto duration = std::chrono::high_resolution_clock::now() - start;
  ROS_INFO_STREAM("Pipeline took:" << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms");
}

Eigen::Matrix3Xd DynamicMappingRos::rosToEigen(const sensor_msgs::PointCloud2& ros) {
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pcl::fromROSMsg(ros, pclCloud);
  return pclCloud.getMatrixXfMap().cast<double>();
}

void DynamicMappingRos::debugOutput() {
  jsk_recognition_msgs::BoundingBoxArray bboxArr;
  std::vector<jsk_recognition_msgs::BoundingBox> trackBoxes;
  std::vector<jsk_recognition_msgs::BoundingBox> detectionBoxes;
  visualization_msgs::MarkerArray markerArray;
  for (const auto& box : filter_->getDetectionBoxes()) {
    detectionBoxes.push_back(generateBoundingBoxMsg(box));
  }
  for (const auto& box : filter_->getTrackBoxes()) {
    trackBoxes.push_back(generateBoundingBoxMsg(box));
    markerArray.markers.push_back(createMarker(box));
  }
  bboxArr.header.frame_id = "map";
  bboxArr.boxes = detectionBoxes;
  bboxPub_.publish(bboxArr);
  bboxArr.header.frame_id = "map";
  bboxArr.boxes = trackBoxes;
  trackBoxesPub_.publish(bboxArr);
  markerPub_.publish(markerArray);
}

cv::Mat DynamicMappingRos::undistortMask(const cv::Mat& image) {
  cv::Mat undistorted;
  {
    std::lock_guard<std::mutex> guard{camInfoLock_};
    cv::remap(image, undistorted, map1_, map2_, cv::INTER_LINEAR);
  }
  return undistorted;
}

void DynamicMappingRos::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camInfo) {
  {
    std::lock_guard<std::mutex> lock{camInfoLock_};
    if (!((camInfo->D == camInfo_.D) && (camInfo->K == camInfo_.K) && (camInfo->P == camInfo_.P))) {
      imageHeight_ = camInfo->height;
      imageWidth_ = camInfo->width;
      auto distCoeffs = camInfo->D;
      auto intrinsics = camInfo->K;
      auto projection = camInfo->P;

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
void DynamicMappingRos::maskCallback(const sensor_msgs::PointCloud2& noGnd, const sensor_msgs::Image& mask) {
  geometry_msgs::TransformStamped lidarToWorld, worldToCamera;
  try {
    lidarToWorld = tfBuffer_.lookupTransform("map", "os_sensor", ros::Time(0));
    worldToCamera = tfBuffer_.lookupTransform("camMainView", "map", ros::Time(0));
  } catch (tf2::LookupException& e) {
    ROS_ERROR("%s", e.what());
    return;
  }
  sensor_msgs::PointCloud2 noGndWorld;
  tf2::doTransform(noGnd, noGndWorld, lidarToWorld);
  Eigen::Matrix3Xd noGroundCloud = rosToEigen(noGndWorld);

  // do the copying ROS <-> OpenCV ourselves because cv_bridge does weird stuff to the mask
  cv::Mat image;
  cv::Mat(mask.height, mask.width, CV_8UC3, const_cast<uchar*>(&mask.data[0]), mask.step).copyTo(image);

  if (mask.encoding == "rgb8") {
    // manually swap red and blue channels, we are expecting bgr8 image
    cv::Mat rgb[3], tmp;
    cv::split(image, rgb);
    tmp = rgb[0];
    rgb[0] = rgb[2];
    rgb[2] = tmp;
    cv::merge(rgb, 3, image);
  }

  cv::Mat segmentationMask = shouldUndistort_ ? undistortMask(image) : image;
  filter_->setCameraTransform(tf2::transformToEigen(worldToCamera));
  filter_->updateLabels(noGroundCloud, segmentationMask, noGnd.header.stamp.toSec());
  if (verbose_) {
    pub_.publish(generateClusterPointCloud(filter_->getLabelledClusters()));
  }
}

std::vector<dynamic_mapping_msgs::Track> DynamicMappingRos::generateTrackMsgs() {
  std::vector<dynamic_mapping_msgs::Track> tracks;
  auto filterTracks = filter_->getTracks();
  for (const auto& track : filterTracks) {
    if (track.getLabel() < 0) continue;
    dynamic_mapping_msgs::Track trackMsg;
    auto trackState = track.getState();

    trackMsg.pose.position.x = trackState.state[0];
    trackMsg.pose.position.y = trackState.state[2];
    trackMsg.pose.position.z = trackState.state[4];

    trackMsg.pose.orientation.w = 1;

    trackMsg.velocity.linear.x = trackState.state[1];
    trackMsg.velocity.linear.y = trackState.state[3];
    trackMsg.velocity.linear.z = trackState.state[5];

    trackMsg.dimensions.x = trackState.state[6];
    trackMsg.dimensions.y = trackState.state[7];
    trackMsg.dimensions.z = trackState.state[8];

    trackMsg.label = track.getLabel();
    trackMsg.uuid = track.getUuid();
    tracks.push_back(trackMsg);
  }
  return tracks;
}
