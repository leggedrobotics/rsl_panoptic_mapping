// SemanticMap.h - Header for SemanticMap class in M545 navigation simulation
// Processes dynamic and static point clouds to generate a grid map with semantic, traversability, and occupancy layers.
// For use in robotic navigation and environment understanding.
#include <grid_map_msgs/GridMap.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <mutex>
#include <map>
#include <string>
#include <vector>

namespace panoptic_gridmap {

class SemanticMap {
 public:
  SemanticMap(ros::NodeHandle& nh, const std::string& configYamlPath);
  ~SemanticMap();

  // Callback to process incoming point clouds and update the map.
  void mapCallback(const sensor_msgs::PointCloud2ConstPtr& dynamicPcl, const sensor_msgs::PointCloud2ConstPtr& staticPcl);

  // Asynchronous map update from point cloud data.
  void updateMap(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, grid_map::GridMap& map);
  void updateSingleScanStuffMap(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, grid_map::GridMap& map);
  void updateSingleScanThingsMap(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, grid_map::GridMap& map);

  // Fuses various map layers into a single coherent map.
  void fuseLayers(grid_map::GridMap& map, grid_map::GridMap& tempMap);

  // Publishes the final semantic map.
  void publishSemanticMap(ros::Time time);

 private:
  ros::NodeHandle nh_;
  ros::Publisher semanticMapPublisher_;
  ros::Publisher textMarkerPublisher_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> subDynamicPcl_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> subStaticPcl_;

  // Synchronizer for point cloud processing.
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> PclExactSyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<PclExactSyncPolicy>> pclExactSyncPtr_;

  tf::TransformListener tfListener_;

  // Utility functions for marker publishing and parameter loading.
  void publishTextGridMapMarkers(std::vector<grid_map::Position> positions, std::vector<std::string> texts);
  void publishTextForOccupiedCells();
  void loadParameters(const std::string& configYamlPath);
  YAML::Node config_;

  // Semantic and cost decoding utilities.
  std::map<int, std::string> idToLabel_;
  std::map<std::string, int> labelToId_;
  std::string getLabel(int id, std::string type);
  std::map<std::string, float> labelToCostDictionary_;
  float getCost(const std::string& label);
  bool usingIds_;
  std::map<std::string, int> labelToOccupancyDictionary_;
  int getOccupancy(const std::string& label);
  int parseThingsId(int id);
  int parseStuffId(int id);

  grid_map::GridMap map_;

  std::mutex map_mutex_;
};

}  // namespace panoptic_gridmap
