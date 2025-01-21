#include "ground_remover/elevation_map_ground_remover.hpp"
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace ground_remover;

ElevationMapGroundRemover::ElevationMapGroundRemover(const std::string& configFilePath, const float leaf_size, rclcpp::Node::SharedPtr node) {
  _groundRemover = std::make_shared<ground_removal::ElevationMapGroundPlaneRemover>(node);
  ground_removal::ElevationMapGroundPlaneRemoverParam groundRemovalParams(node->get_logger()); // have to give node logger in there
  ground_removal::loadParameters(configFilePath, &groundRemovalParams, node->get_logger());
  _groundRemover->setParameters(groundRemovalParams);
  _leaf_size = leaf_size;
}

void ElevationMapGroundRemover::removeGround(PointCloud& in, PointCloud& out) {
  pcl::Indices indices;
  in.is_dense = false;
  //std::cout << "Input cloud size: " << in.size() << std::endl;
  //std::cout << "Output cloud size: " << out.size() << std::endl;

  pcl::removeNaNFromPointCloud(in, out, indices);
  //std::cout << "Input cloud size without nan: " << in.size() << std::endl;
  //std::cout << "Output cloud size: " << out.size() << std::endl;

  removeRangeGreaterThan(out, out, 50.f);
  //std::cout << "Output cloud size after removing range greater than 50: " << out.size() << std::endl;
  const PointCloud::Ptr voxelCloud(new PointCloud);
  voxelizeCloud(out, *voxelCloud, _leaf_size);
  //std::cout << "voxelizedCloud size: " << voxelCloud->size() << std::endl;
  _groundRemover->setInputCloudPtr(voxelCloud);
  _groundRemover->setFilterCloudPtr(out.makeShared());
  _groundRemover->removeGroundPlane(); // problematic function 
  PointCloud::ConstPtr noGroundCloud = _groundRemover->getCloudWithoutGroundPlanePtr();
  out = *noGroundCloud;
  }

void ElevationMapGroundRemover::removeRangeGreaterThan(const PointCloud& cloud_in, PointCloud& cloud_out, float range) {
  std::size_t j = 0;
  for (std::size_t i = 0; i < cloud_in.size(); ++i) {
    auto dist = std::sqrt(cloud_in[i].x * cloud_in[i].x + cloud_in[i].y * cloud_in[i].y + cloud_in[i].z * cloud_in[i].z);
    if (dist - range > 0 || dist < 0.001f) {
      continue;
    }
    cloud_out[j] = cloud_in[i];
    j++;
  }
  if (j != cloud_out.size()) {
    cloud_out.resize(j);
  }
  cloud_out.height = 1;
  cloud_out.width = static_cast<std::uint32_t>(j);
}

void ElevationMapGroundRemover::voxelizeCloud(const PointCloud& cloud_in, PointCloud& cloud_out, float leaf_size) {
  pcl::VoxelGrid<Point> filter;
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.setInputCloud(cloud_in.makeShared());
  filter.filter(cloud_out);
}