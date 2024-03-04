#include <sensor_msgs/PointCloud2.h>
#include "ground_remover.h"

#include "ground_plane_removal/ElevationMapGroundPlaneRemover.hpp"
#include "ground_plane_removal/Parameters.hpp"
#include "tree_detection_ros/creators.hpp"

namespace ground_remover {
class ElevationMapGroundRemover : public GroundRemover {
 public:
  ElevationMapGroundRemover(const std::string& configFilePath, const float leaf_size);
  void removeGround(PointCloud& in, PointCloud& out) override;

 private:
  std::shared_ptr<ground_removal::ElevationMapGroundPlaneRemover> _groundRemover;
  float _leaf_size;
  void removeRangeGreaterThan(const PointCloud& cloud_in, PointCloud& cloud_out, float range);
  void voxelizeCloud(const PointCloud& cloud_in, PointCloud& cloud_out, float leaf_size);
};
}  // namespace ground_remover