/*
 * Parameters.hpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */

#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include "grid_map_pcl/PclLoaderParameters.hpp"

namespace ground_removal {


struct GroundPlaneCropBoxParameters
{
  double minX_ = -50.0;
  double maxX_= 50.0;
  double minY_= -50.0;
  double maxY_= 50.0;
  double minZ_= -10.0;
  double maxZ_= 10.0;

};


struct GroundPlaneRemoverParam {
	GroundPlaneCropBoxParameters cropBox_;
};

struct ElevationMapGroundPlaneRemoverParam : public GroundPlaneRemoverParam {
	grid_map::grid_map_pcl::PclLoaderParameters pclConverter_;
	double minHeightAboveGround_ = 0.1;
	double maxHeightAboveGround_ = 8.0;
	double medianFilteringRadius_ = 2.0;
	int medianFilterDownsampleFactor_  = 1;
	bool isUseMedianFiltering_ = true;


  // Constructor
  ElevationMapGroundPlaneRemoverParam(
	const rclcpp::Logger& node_logger,
    double minHeightAboveGround = 0.1,
    double maxHeightAboveGround = 8.0,
    double medianFilteringRadius = 2.0,
    int medianFilterDownsampleFactor = 1,
    bool isUseMedianFiltering = true)
    : pclConverter_(node_logger),
      minHeightAboveGround_(minHeightAboveGround),
      maxHeightAboveGround_(maxHeightAboveGround),
      medianFilteringRadius_(medianFilteringRadius),
      medianFilterDownsampleFactor_(medianFilterDownsampleFactor),
      isUseMedianFiltering_(isUseMedianFiltering) {}
};
 
};

std::ostream& operator<<(std::ostream& out, const GroundPlaneCropBoxParameters& p);
void loadParameters(const YAML::Node &node, GroundPlaneCropBoxParameters *p);

} // namespace ground_removal