#include "ground_remover/elevation_map_ground_remover.h"
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace ground_remover;

ElevationMapGroundRemover::ElevationMapGroundRemover(const std::string& configFilePath, const float leaf_size) {
  _groundRemover = std::make_shared<ground_removal::ElevationMapGroundPlaneRemover>();
  ground_removal::ElevationMapGroundPlaneRemoverParam groundRemovalParams;
  ground_removal::loadParameters(configFilePath, &groundRemovalParams);
  _groundRemover->setParameters(groundRemovalParams);
  _leaf_size = leaf_size;
}

void ElevationMapGroundRemover::removeGround(PointCloud& in, PointCloud& out) {
  pcl::Indices indices;
  in.is_dense = false;
  pcl::removeNaNFromPointCloud(in, out, indices);
  removeRangeGreaterThan(out, out, 50.f);
  const PointCloud::Ptr voxelCloud(new PointCloud);
  voxelizeCloud(out, *voxelCloud, _leaf_size);
  _groundRemover->setInputCloudPtr(voxelCloud);
  _groundRemover->setFilterCloudPtr(out.makeShared());
  _groundRemover->removeGroundPlane();
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