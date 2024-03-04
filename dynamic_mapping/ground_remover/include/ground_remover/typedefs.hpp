#ifndef GROUND_REMOVER_TYPEDEFS_H_
#define GROUND_REMOVER_TYPEDEFS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ground_remover {
using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
}  // namespace ground_remover

#endif