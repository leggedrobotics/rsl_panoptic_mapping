#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "elevation_map_ground_remover.h"
#include "ground_remover.h"

namespace ground_remover {
class GroundRemoverRos {
 public:
  enum ALGO { GRID_MAP = 0, PATCHWORK };
  GroundRemoverRos(const ros::NodeHandlePtr& nh, const std::string& inputTopic, const std::string& outputTopic,
                   const std::string& configFilePath, float leaf_size);
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc);

 private:
  ros::NodeHandlePtr nh_;
  ros::Subscriber cloudSubscriber_;
  ros::Publisher noGroundCloudPublisher_;
  std::unique_ptr<GroundRemover> groundRemover_;
};
}  // namespace ground_remover
