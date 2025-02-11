//
// Created by lorenzo on 2/2/23.
// This is the ros node that instantiates the SemanticMap class and receives the point clouds from the dynamic mapping node.
//
#include <panoptic_gridmap/SemanticMap.h>

int main(int argc, char** argv) {
  // Initialize the ROS node.
  ros::init(argc, argv, "semantic_map_node");
  ros::NodeHandle nh_;
  // load the config path from the parameter server
  std::string configYamlPath;
  nh_.param<std::string>("/panoptic_gridmap/config_dir", configYamlPath, "config.yaml");
  // Create the SemanticMap object and pass the node handle and config path to the constructor.
  panoptic_gridmap::SemanticMap semantic_map(nh_, configYamlPath);
  // Spin the node.
  ros::spin();
  return 0;
}