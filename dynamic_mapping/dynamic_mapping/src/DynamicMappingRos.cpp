#include "dynamic_mapping/DynamicMappingRos.h"
#include <cv_bridge/cv_bridge.h>
#include <dynamic_mapping_msgs/msg/track_array.hpp>
#include <jsk_recognition_msgs/msg/bounding_box_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/msg/marker_array.hpp>

size_t Detection::id_ = 0;
DynamicMappingRos::DynamicMappingRos(
    const rclcpp::Node::SharedPtr& node,
    const GeneralParameters& generalParameters,
    const ObjectFilterParameters& objectFilterParameters)
    : node_(node), verbose_(generalParameters.verbose), tfBuffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)) {
  listener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_);
  if (verbose_) {
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(generalParameters.clusterTopic, 1);
    bboxPub_ = node_->create_publisher<jsk_recognition_msgs::msg::BoundingBoxArray>(generalParameters.detectionTopic, 1);
    trackBoxesPub_ = node_->create_publisher<jsk_recognition_msgs::msg::BoundingBoxArray>(generalParameters.trackTopic, 1);
  }
  cloudPubStatic_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(generalParameters.outputTopic + "/static", 1);
  cloudPubDynamic_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(generalParameters.outputTopic + "/dynamic", 1);
  trackPub_ = node_->create_publisher<dynamic_mapping_msgs::msg::TrackArray>("/tracks", 1);
  markerPub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/markers", 1);

  pub_check_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("check_pc", 1);


  // clang-format off
  //cameraIntrinsics_ = (cv::Mat_<double>(3, 3) <<   767.239452,        0.0, 814.414996,
  //                                                        0.0, 767.168172, 597.978256,
  //                                                        0.0,        0.0,        1.0);
  cameraIntrinsics_ = (cv::Mat_<double>(3, 3) << 732.999, 0.0, 320, // Isaac Sim
                                                  0.0, 732.999, 240,
                                                  0.0, 0.0, 1.0);
  cameraProjection_ <<   732.999, 0.0, 320.0, 0.0,
                                0.0, 732.999, 240.0, 0.0,
                                0.0,        0.0,        1.0, 0.0;
  // clang-format on
  distorsionCoefficients_ = (cv::Mat_<double>(1, 5) << -0.044303, 0.006917, -0.000472, -0.000009, 0);
  cv::initUndistortRectifyMap(cameraIntrinsics_, distorsionCoefficients_, cv::Mat_<double>::eye(3, 3), cameraIntrinsics_,
                              cv::Size(imageWidth_, imageHeight_), CV_16SC2, map1_, map2_);

  filter_ = std::make_unique<MovingObjectsFilter>(objectFilterParameters);
  filter_->updateCameraProjection(cameraProjection_);
  cameraInfoSubscriber_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      generalParameters.cameraInfoTopic, 1, std::bind(&DynamicMappingRos::cameraInfoCallback, this, std::placeholders::_1));
}

void DynamicMappingRos::callback(
    sensor_msgs::msg::PointCloud2 raw, sensor_msgs::msg::PointCloud2 noGnd, sensor_msgs::msg::CompressedImage cam) {
  
   if (maskCallback_called_==false){
    std::cout<<"maskCallback not called"<<std::endl;
    return;
   }
  

  // Convert the compressed image data to a cv::Mat
  cv::Mat decodedImage = cv::imdecode(cv::Mat(cam.data), cv::IMREAD_COLOR);
  if (decodedImage.empty()) {
      std::cerr << "Failed to decode compressed image!" << std::endl;
      return;
  }
  // Get the height and width of the image
  int height = decodedImage.rows;
  int width = decodedImage.cols;

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

    //std::cout<<"callback"<<std::endl;
    if (raw.height == 0 || raw.width == 0) {
          return;
      }
    if (noGnd.height == 0 || noGnd.width == 0) {
          return;
      }


  geometry_msgs::msg::TransformStamped lidarToWorld, worldToCamera, worldToLidar, shovelFrame;
  try {
    lidarToWorld = tfBuffer_.lookupTransform("map", "os_sensor", tf2::TimePointZero);
    worldToCamera = tfBuffer_.lookupTransform("camMainView", "map", tf2::TimePointZero);
    worldToLidar = tfBuffer_.lookupTransform("os_sensor", "map", tf2::TimePointZero);
    shovelFrame = tfBuffer_.lookupTransform("map", "ROTO_BASE", tf2::TimePointZero); // TODO: Change to shovel frame
  } catch (tf2::TransformException& e) {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    return;
  }
  sensor_msgs::msg::PointCloud2 rawWorld, noGndWorld;

  tf2::doTransform(raw, rawWorld, lidarToWorld);
  tf2::doTransform(noGnd, noGndWorld, lidarToWorld);

  Eigen::Matrix3Xd fullCloud = rosToEigen(rawWorld);
  Eigen::Matrix3Xd noGroundCloud = rosToEigen(noGndWorld);


  filter_->setCameraTransform(tf2::transformToEigen(worldToCamera.transform));
  filter_->filterObjects(fullCloud, noGroundCloud, node_->now().seconds());  
  filter_->setShovelFrame(tf2::transformToEigen(shovelFrame.transform));

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
  RCLCPP_INFO(node_->get_logger(), "Pipeline took: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
}

Eigen::Matrix3Xd DynamicMappingRos::rosToEigen(const sensor_msgs::msg::PointCloud2& ros) {
  
  // Sanity check
  if (ros.data.size() != ros.height * ros.width * ros.point_step) {
      std::cerr << "Warning: Data size mismatch with expected dimensions!" << std::endl;
  }
  
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pcl::fromROSMsg(ros, pclCloud);

  auto res = pclCloud.getMatrixXfMap(3,4,0);//(3, 4, 0); // TODO: Check if this is correct

  return res.cast<double>(); //dim=3, stride=4 and offset=0

}




void DynamicMappingRos::debugOutput() {
  jsk_recognition_msgs::msg::BoundingBoxArray bboxArr;
  std::vector<jsk_recognition_msgs::msg::BoundingBox> trackBoxes;
  std::vector<jsk_recognition_msgs::msg::BoundingBox> detectionBoxes;
  visualization_msgs::msg::MarkerArray markerArray;
  for (const auto& box : filter_->getDetectionBoxes()) {
    detectionBoxes.push_back(generateBoundingBoxMsg(box));
  }
  for (const auto& box : filter_->getTrackBoxes()) {
    trackBoxes.push_back(generateBoundingBoxMsg(box));
    markerArray.markers.push_back(createMarker(box));
  }
  bboxArr.header.frame_id = "map";
  bboxArr.boxes = detectionBoxes;
  bboxPub_->publish(bboxArr);
  bboxArr.header.frame_id = "map";
  bboxArr.boxes = trackBoxes;
  trackBoxesPub_->publish(bboxArr);
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

    std::cout<<"Camera Info Callback"<<std::endl;
    std::lock_guard<std::mutex> lock{camInfoLock_};
    if (!((camInfo->d == camInfo_.d) && (camInfo->k == camInfo_.k) && (camInfo->p == camInfo_.p))) {
      // goes through the firs step      
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
  
  std::cout << "Mask Callback" << std::endl;
  maskCallback_called_ = true;
  geometry_msgs::msg::TransformStamped lidarToWorld, worldToCamera;
  try {
    lidarToWorld = tfBuffer_.lookupTransform("map", "os_sensor", tf2::TimePointZero);
    worldToCamera = tfBuffer_.lookupTransform("camMainView", "map", tf2::TimePointZero);
  } catch (tf2::TransformException& e) {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    return;
  }
  sensor_msgs::msg::PointCloud2 noGndWorld;
  tf2::doTransform(noGnd, noGndWorld, lidarToWorld);
  Eigen::Matrix3Xd noGroundCloud = rosToEigen(noGndWorld);

  if (noGnd.height == 0 || noGnd.width == 0) {
          return;
      }


  // do the copying ROS <-> OpenCV ourselves because cv_bridge does weird stuff to the mask
  cv::Mat image;
  cv::Mat(mask.height, mask.width, CV_8UC3, const_cast<uchar*>(&mask.data[0]), mask.step).copyTo(image);
  if (mask.data.size() < mask.step * mask.height) {
      throw std::runtime_error("Image data size is inconsistent with step and height.");
  }
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
  filter_->setCameraTransform(tf2::transformToEigen(worldToCamera.transform));
  filter_->updateLabels(noGroundCloud, segmentationMask, noGnd.header.stamp.sec + noGnd.header.stamp.nanosec * 1e-9);
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
