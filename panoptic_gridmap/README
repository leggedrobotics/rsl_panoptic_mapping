# Panoptic Grid Map

The M545 Semantic Map ROS package is designed for generating semantic maps from dynamic and static point clouds. It incorporates semantic and instance information about movable and static objects, respectively, to create a comprehensive grid map. This map includes layers for semantic information, traversability cost, and occupancy, facilitating advanced navigation and environment understanding in robotic applications.

## Features

- **Semantic Layer Generation:** Incorporates semantic and instance information from point clouds.
- **Traversability Cost Layer:** Calculates and assigns costs for navigation based on the semantic information.
- **Occupancy Map:** Generates an occupancy grid based on static and dynamic objects in the environment.
- **Filtering and Fusion:** Applies median filters and fuses layers for enhanced map quality.
- **ROS Integration:** Fully integrated with ROS for easy deployment in robotic systems.

## Prerequisites

- ROS (Robot Operating System) [Noetic]
- PCL (Point Cloud Library)
- grid_map ROS package
- message_filters ROS package
- tf ROS package
- pcl_ros package
- yaml-cpp

## Installation
Clone the repository into your ROS workspace's `src` directory and build it.

## Usage

1. **Configuration:** Modify the `config.yaml` file in the `config` directory to suit your application requirements. This file contains parameters for map generation, filter settings, and topic names.

2. **Launch the Node:** roslaunch panoptic_gridmap panoptic_gridmap.launch

This will start the `panoptic_gridmap` node with the default settings specified in the launch and configuration files. You can override these settings using launch file arguments if necessary.

## Topics

- **Subscribed Topics:**
- `/new_cloud/dynamic` (sensor_msgs/PointCloud2): Dynamic point cloud topic.
- `/new_cloud/static` (sensor_msgs/PointCloud2): Static point cloud topic.

- **Published Topics:**
- `/semantic_map` (grid_map_msgs/GridMap): The generated semantic grid map.

## Node Details

- **`panoptic_gridmap` Node:** Main node responsible for generating the semantic map. It subscribes to dynamic and static point cloud topics, processes the data, and publishes a grid map with multiple layers.

## Configuration

The package can be configured via the `config.yaml` file, where parameters such as frame ID, map size, resolution, and filter settings can be adjusted.

## Contributing

Contributions to improve `panoptic_gridmap` are welcome. Please submit pull requests with your proposed changes or enhancements.

## License

Specify the license under which this package is released, such as MIT, GPL, etc.

