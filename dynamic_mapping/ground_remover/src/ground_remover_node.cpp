#include <ros/ros.h>
#include "ground_remover/ground_remover_ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_remover_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  const std::string configFilePath = nh->param<std::string>("config_filepath", "");
  const std::string input_topic = nh->param<std::string>("input_topic", "/ouster_points_self_filtered");
  const std::string outputTopic = "no_ground_cloud";
  const float leaf_size = nh->param<float>("voxel_size", 0.15);
  ground_remover::GroundRemoverRos groundRemover(nh, input_topic, outputTopic, configFilePath, leaf_size);
  ros::spin();
}
