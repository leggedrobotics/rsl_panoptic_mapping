#ifndef DYNAMIC_MAPPING_HELPERS_H_
#define DYNAMIC_MAPPING_HELPERS_H_

#include <jsk_recognition_msgs/BoundingBox.h>
#include <open3d/geometry/BoundingVolume.h>
#include <open3d/geometry/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <vector>
#include "dynamic_mapping/BoundingBox.h"
#include "dynamic_mapping/Parameters.h"
#include "dynamic_mapping/tracking/Detection.h"

open3d::geometry::PointCloud eigenToOpen3d(const Eigen::Matrix3Xd& pts);
std::vector<int> getClusterIndices(const std::vector<int>& clusters, int clusterId);
template <typename T>
std::vector<T> getUnique(const std::vector<T>& vec);
template <typename T>
std::unordered_map<T, int> getOccurencePerElement(const std::vector<T>& vec);
open3d::geometry::AxisAlignedBoundingBox getBoundingBox(const Eigen::Matrix3Xd& pts);
jsk_recognition_msgs::BoundingBox generateBoundingBoxMsg(const BoundingBox& bbox);
Eigen::Matrix<double, 9, 1> stateFromDetection(const Detection& det);
double mahalanobis(const Eigen::VectorXd& state1, const Eigen::VectorXd& state2, const Eigen::MatrixXd& cov1);
sensor_msgs::PointCloud2 generateClusterPointCloud(const std::vector<Eigen::Matrix4Xd>& labelledClusters);
sensor_msgs::PointCloud2 eigenToRos(const Eigen::Matrix4Xd& cloud, const std::string& frame_id);

void loadParameters(const std::string& file, MessageProcessorParameters* params);
void loadParameters(const std::string& file, ObjectFilterParameters* params);
void loadParameters(const std::string& file, GeneralParameters* params);
double minDistanceToFrame(const BoundingBox& box, const Eigen::Isometry3d& frame);
visualization_msgs::Marker createMarker(const BoundingBox& box);
Eigen::Matrix3Xd removeNans(const Eigen::Matrix3Xd& in);

const std::unordered_map<int, std::string> idToLabel = {{255, "unobserved"},    {254, "incomplete"},
                                                        {0, "unknown"},         {1, "person"},
                                                        {2, "bicycle"},         {3, "car"},
                                                        {4, "motorcycle"},      {5, "train"},
                                                        {6, "truck"},           {7, "boat"},
                                                        {8, "bridge"},          {9, "buildingOtherMerged"},
                                                        {10, "gravel"},         {11, "railroad"},
                                                        {12, "road"},           {13, "pavementMerged"},
                                                        {14, "dirtMerged"},     {15, "sand"},
                                                        {16, "snow"},           {17, "floorOtherMerged"},
                                                        {18, "grassMerged"},    {19, "wallOtherMerged"},
                                                        {20, "rockMerged"},     {21, "waterOther"},
                                                        {22, "treeMerged"},     {23, "fenceMerged"},
                                                        {24, "skyOtherMerged"}, {25, "floorConcrete"},
                                                        {26, "container"},      {27, "selfArm"},
                                                        {28, "selfLeg"},        {29, "stone"},
                                                        {30, "gravelPile"},     {31, "sandPile"},
                                                        {32, "unknownSemSeg"},  {33, "bucket"},
                                                        {34, "gripper"}};

#include "dynamic_mapping/helpers_impl.hpp"

#endif