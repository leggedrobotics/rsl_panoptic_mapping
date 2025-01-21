/*
 * Parameters.cpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */
#include "ground_plane_removal/Parameters.hpp"
#include <iomanip>

namespace ground_removal {

void loadParameters(const YAML::Node &node, GroundPlaneCropBoxParameters *p) {
	p->minX_ = node["crop_box_minX"].as<double>();
	p->maxX_ = node["crop_box_maxX"].as<double>();
	p->minY_ = node["crop_box_minY"].as<double>();
	p->maxY_ = node["crop_box_maxY"].as<double>();
	p->minZ_ = node["crop_box_minZ"].as<double>();
	p->maxZ_ = node["crop_box_maxZ"].as<double>();
}


void loadParameters(const std::string &filename, ground_removal::ElevationMapGroundPlaneRemoverParam *p, const rclcpp::Logger & node_logger) {

	YAML::Node node = YAML::LoadFile(filename);
	auto groundRemoval = node["ground_plane_removal"]["elevation_map"];
	grid_map::grid_map_pcl::PclLoaderParameters pclLoaderParam(node_logger);
	pclLoaderParam.handleYamlNode(groundRemoval);
	p->pclConverter_ = pclLoaderParam;

	p->medianFilteringRadius_  = groundRemoval["median_filtering_radius"].as<double>();
	p->medianFilterDownsampleFactor_ = groundRemoval["median_filter_points_downsample_factor"].as<int>();
	p->minHeightAboveGround_ = groundRemoval["min_height_above_ground"].as<double>();
	p->maxHeightAboveGround_ = groundRemoval["max_height_above_ground"].as<double>();
	p->isUseMedianFiltering_ = groundRemoval["is_use_median_filter"].as<bool>();
}

std::ostream& operator<<(std::ostream& out, const GroundPlaneCropBoxParameters& p) {
  out << "┌────────────────────────────────────────────────────┐\n";
  out << "│                 GroundPlaneCropBoxParameters       │\n";
  out << "├─────────────────────────────────┬──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << " minX"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.minX_  << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "max X"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.minX_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "min Y"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.minY_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "max Y"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.maxY_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "min Z"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.minZ_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "max Z"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.maxZ_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";


  return out;
}

} // namespace ground_removal


