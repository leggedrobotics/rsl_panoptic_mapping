#include "ground_remover/ground_remover_ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <chrono>

using namespace ground_remover;

GroundRemoverRos::GroundRemoverRos(const ros::NodeHandlePtr& nh, const std::string& inputTopic, const std::string& outputTopic,
                                   const std::string& configFilePath, float leaf_size)
    : nh_(nh) {
  cloudSubscriber_ = nh_->subscribe(inputTopic, 1, &GroundRemoverRos::cloudCallback, this);
  noGroundCloudPublisher_ = nh_->advertise<sensor_msgs::PointCloud2>(outputTopic, 1);
  groundRemover_ = std::make_unique<ElevationMapGroundRemover>(configFilePath, leaf_size);
}

void GroundRemoverRos::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc) {
  PointCloud inCloud;
  PointCloud outCloud;
  auto start = std::chrono::high_resolution_clock::now();
  pcl::fromROSMsg(*pc, inCloud);
  groundRemover_->removeGround(inCloud, outCloud);
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(outCloud, outMsg);
  outMsg.header = pc->header;
  noGroundCloudPublisher_.publish(outMsg);
  auto end = std::chrono::high_resolution_clock::now();
  ROS_INFO_STREAM("Ground removal: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms");
}
