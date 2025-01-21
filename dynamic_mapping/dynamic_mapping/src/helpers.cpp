#include "dynamic_mapping/helpers.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>

// Convert Eigen points to Open3D PointCloud
open3d::geometry::PointCloud eigenToOpen3d(const Eigen::Matrix3Xd& pts) {
  std::vector<Eigen::Vector3d> pointsVector;
  for (auto col : pts.colwise()) pointsVector.push_back(col.transpose());
  return open3d::geometry::PointCloud(pointsVector);
}

// Get cluster indices by ID
std::vector<int> getClusterIndices(const std::vector<int>& clusters, int clusterId) {
  std::vector<int> results;
  auto it = clusters.begin();
  while ((it = std::find_if(it, clusters.end(), [&](const int& el) { return el == clusterId; })) != clusters.end()) {
    results.push_back(std::distance(clusters.begin(), it));
    it++;
  }
  return results;
}

// Get axis-aligned bounding box
open3d::geometry::AxisAlignedBoundingBox getBoundingBox(const Eigen::Matrix3Xd& pts) {
  open3d::geometry::PointCloud pcl = eigenToOpen3d(pts);
  return pcl.GetAxisAlignedBoundingBox();
}

// Generate bounding box message
jsk_recognition_msgs::msg::BoundingBox generateBoundingBoxMsg(const BoundingBox& bbox) {
  jsk_recognition_msgs::msg::BoundingBox bboxMsg;
  bboxMsg.pose.position.x = bbox.center.x();
  bboxMsg.pose.position.y = bbox.center.y();
  bboxMsg.pose.position.z = bbox.center.z();

  bboxMsg.dimensions.x = bbox.extent.x();
  bboxMsg.dimensions.y = bbox.extent.y();
  bboxMsg.dimensions.z = bbox.extent.z();

  bboxMsg.label = bbox.label;
  bboxMsg.value = static_cast<float>(bbox.extentVelocity.norm());

  bboxMsg.header.frame_id = "map";
  return bboxMsg;
}

// Convert detection to state
Eigen::Vector<double, 9> stateFromDetection(const Detection& det) {
  Eigen::Vector<double, 9> state{det.centroid[0], 0, det.centroid[1], 0, det.centroid[2], 0, det.extent[0], det.extent[1], det.extent[2]};
  return state;
}

// Mahalanobis distance calculation
double mahalanobis(const Eigen::VectorXd& state1, const Eigen::VectorXd& state2, const Eigen::MatrixXd& cov1) {
  auto delta = state1 - state2;
  auto vi = cov1.inverse();
  auto mahalanobisSqr = delta.transpose() * (vi * delta);
  return sqrt(mahalanobisSqr(0, 0));
}

// Generate labeled cluster PointCloud2
sensor_msgs::msg::PointCloud2 generateClusterPointCloud(const std::vector<Eigen::Matrix4Xd>& labelledClusters) {  int numCols = 0;
  for (const auto& mat : labelledClusters) {
    numCols += mat.cols();
  }
  Eigen::Matrix4Xd concatMat(4, numCols);
  int colOffset = 0;
  std::vector<int> clusters;
  for (int i = 0; i < labelledClusters.size(); i++) {
    int cols = labelledClusters[i].cols();
    concatMat.middleCols(colOffset, cols) = labelledClusters[i];
    std::vector<int> cluster(cols, i);
    clusters.insert(clusters.end(), cluster.begin(), cluster.end());
    colOffset += cols;
  }
  sensor_msgs::msg::PointCloud2 outCloud;
  outCloud.height = 1;
  outCloud.width = concatMat.cols();
  outCloud.header.frame_id = "map";
  outCloud.header.stamp = rclcpp::Clock().now();

  sensor_msgs::PointCloud2Modifier outCloudModifier(outCloud);
  outCloudModifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                        "z", 1, sensor_msgs::msg::PointField::FLOAT32, "cluster", 1, sensor_msgs::msg::PointField::INT32,
                                        "label", 1, sensor_msgs::msg::PointField::INT32);

  sensor_msgs::PointCloud2Iterator<float> x_iter(outCloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_iter(outCloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_iter(outCloud, "z");
  sensor_msgs::PointCloud2Iterator<int> cluster_iter(outCloud, "cluster");
  sensor_msgs::PointCloud2Iterator<int> label_iter(outCloud, "label");
  Eigen::RowVectorXd x, y, z;
  Eigen::RowVectorXi label;
  x = concatMat.row(0);
  y = concatMat.row(1);
  z = concatMat.row(2);
  label = concatMat.cast<int>().row(3);

  for (int i = 0; x_iter != x_iter.end(); ++i, ++x_iter, ++y_iter, ++z_iter, ++cluster_iter, ++label_iter) {
    *x_iter = x[i];
    *y_iter = y[i];
    *z_iter = z[i];
    *label_iter = label[i];
    *cluster_iter = clusters[i];
  }
  return outCloud;
}

// Eigen to ROS PointCloud2 conversion
sensor_msgs::msg::PointCloud2 eigenToRos(const Eigen::Matrix4Xd& cloud, const std::string& frame_id) {  
  sensor_msgs::msg::PointCloud2 outCloud;
  outCloud.header.frame_id = frame_id;
  outCloud.header.stamp = rclcpp::Clock().now();
  outCloud.height = 1;
  outCloud.width = cloud.cols();

  sensor_msgs::PointCloud2Modifier modifier(outCloud);
  modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "z", 1, sensor_msgs::msg::PointField::FLOAT32, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "label", 1, sensor_msgs::msg::PointField::FLOAT32);

  sensor_msgs::PointCloud2Iterator<float> x_iter(outCloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_iter(outCloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_iter(outCloud, "z");
  sensor_msgs::PointCloud2Iterator<float> label_iter(outCloud, "label");

  for (int i = 0; i < cloud.cols(); ++i, ++x_iter, ++y_iter, ++z_iter, ++label_iter) {
    const Eigen::Vector4d& point = cloud.col(i);
    *x_iter = point(0);
    *y_iter = point(1);
    *z_iter = point(2);
    *label_iter = point(3);
  }
  return outCloud;
}

sensor_msgs::msg::PointCloud2 eigenToRosXYZ(const Eigen::Matrix3Xd& cloud,
                                            const std::string& frame_id)
{
  // Prepare an empty PointCloud2
  sensor_msgs::msg::PointCloud2 outCloud;
  outCloud.header.frame_id = frame_id;
  outCloud.header.stamp = rclcpp::Clock().now();
  outCloud.height = 1;                // unorganized point cloud
  outCloud.width = cloud.cols();      // number of points is #cols
  outCloud.is_bigendian = false;      // standard
  outCloud.is_dense = false;         // or true if no invalid points
  
  // We only have x,y,z - 3 fields total
  sensor_msgs::PointCloud2Modifier modifier(outCloud);
  modifier.setPointCloud2Fields(3,
                                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  
  // Optionally setPointCloud2FieldsByString("xyz") could also be used:
  // modifier.setPointCloud2FieldsByString(1, "xyz");
  
  // We now have iterators for x, y, z
  sensor_msgs::PointCloud2Iterator<float> x_iter(outCloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_iter(outCloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_iter(outCloud, "z");
  
  // Fill the cloud
  for (int i = 0; i < cloud.cols(); ++i, ++x_iter, ++y_iter, ++z_iter) {
    // Each column i is an Eigen::Vector3d: (x, y, z)
    const Eigen::Vector3d& point = cloud.col(i);
    *x_iter = static_cast<float>(point.x());
    *y_iter = static_cast<float>(point.y());
    *z_iter = static_cast<float>(point.z());
  }
  
  return outCloud;
}




// Parameter loading (YAML)
void loadParameters(const std::string& file, MessageProcessorParameters* params) {
  YAML::Node f = YAML::LoadFile(file);
  if (f.IsNull()) {
    throw std::runtime_error("Could not load MessageProcessorParameters!");
  }
  YAML::Node node = f["message_processor"];
  params->matchedMessageTopic = node["matched_message_topic"].as<std::string>();
  params->segmentationMaskTopic = node["segmentation_mask_topic"].as<std::string>();
}

// Parameter loading (YAML)
void loadParameters(const std::string& file, ObjectFilterParameters* params) {
  YAML::Node f = YAML::LoadFile(file);
  if (f.IsNull()) {
    throw std::runtime_error("Could not load ObjectFilterParameters!");
  }

  YAML::Node node = f["object_filter"];

  params->minimumClusterSize = node["minimum_cluster_size"].as<int>();
  params->labels = node["filter_labels"].as<std::vector<int>>();
  params->missedDetectionDistance = node["missed_detection_distance"].as<double>();
  params->deletionCovarThreshold = node["deletion_covar_trace_threshold"].as<double>();
  params->initiationCovarThreshold = node["initiation_covar_trace_threshold"].as<double>();
  params->velocityFilterThreshold = node["velocity_threshold"].as<double>();
  params->boundingBoxMargin = node["bounding_box_scaling"].as<double>();
  params->minimumInitiationPoints = node["minimum_initiation_points"].as<int>();
  params->shovelDistanceThreshold = node["shovel_distance_threshold"].as<double>();
  params->extentVelocityThreshold = node["extent_velocity_threshold"].as<double>();
  params->strictSegmentation = node["strict_segmentation"].as<bool>();
  params->segmentOnGroundRemovedCloud = node["segment_on_ground_removed_cloud"].as<bool>(false);
}

// Parameter loading (YAML)
void loadParameters(const std::string& file, GeneralParameters* params) {
  YAML::Node f = YAML::LoadFile(file);
  if (f.IsNull()) {
    throw std::runtime_error("Could not load GeneralParameters!");
  }
  YAML::Node node = f["dynamic_mapping"];
  params->verbose = node["verbose"].as<bool>();
  params->clusterTopic = node["cluster_topic"].as<std::string>();
  params->detectionTopic = node["detection_topic"].as<std::string>();
  params->trackTopic = node["track_topic"].as<std::string>();
  params->outputTopic = node["output_topic"].as<std::string>();
  params->cameraInfoTopic = node["camera_info_topic"].as<std::string>();
  params->shouldUndistort = node["should_undistort"].as<bool>();
}

// Minimum distance to frame calculation 
double minDistanceToFrame(const BoundingBox& box, const Eigen::Isometry3d& frame) {
  std::vector<Eigen::Vector3d> boxPoints = box.getAxisAlignedBoundingBox().GetBoxPoints();
  double minDistance = 1e6;
  for (const auto& point : boxPoints) {
    auto trans = frame.translation();
    double d = (trans - point).norm();
    if (d < minDistance) {
      minDistance = d;
    }
  }
  return minDistance;
}

// Create a Marker message for visualization
visualization_msgs::msg::Marker createMarker(const BoundingBox& box) {
  visualization_msgs::msg::Marker marker;
  
  // Set the marker header
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  
  // Set the marker namespace and ID
  marker.ns = box.trackUuid;
  marker.id = 0;
  
  // Set marker type and action
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::MODIFY;
  
  // Set marker position
  marker.pose.position.x = box.center.x();
  marker.pose.position.y = box.center.y();
  marker.pose.position.z = box.center.z() + box.extent.z() + 0.5;

  // Set marker scale
  marker.scale.z = 0.2;

  // Set marker color
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;

  // Set marker lifetime
  marker.lifetime = rclcpp::Duration::from_seconds(0.25);

  // Generate marker text
  std::stringstream ss;
  ss << idToLabel.at(box.label) << "\n vel.: " << box.velocity;
  marker.text = ss.str();

  return marker;
}

// Remove NaNs from an Eigen matrix
Eigen::Matrix3Xd removeNans(const Eigen::Matrix3Xd& in) {
  Eigen::Matrix3Xd out;
  for (int i = 0; i < in.cols(); i++) {
    if (!in.col(i).hasNaN()) {
      out.conservativeResize(Eigen::NoChange, out.cols() + 1);
      out.col(out.cols() - 1) = in.col(i);
    }
  }
  return out;
}