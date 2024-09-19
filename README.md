# RSL Panoptic Mapping

![overview.png](docs%2Foverview.png)

RSL Panoptic Mapping is a ROS package for advanced environment understanding and navigation. It uses semantic segmentation and dynamic object tracking to create detailed panoptic maps, distinguishing between movable and static objects.

## Features

- **Semantic Layer Generation:** Generates semantic layers from dynamic and static point clouds.
- **Dynamic Mapping and Tracking:** Separates point clouds into labeled groups for real-time tracking.
- **Traversability Cost Layer:** Assigns navigation costs based on semantic information.
- **Occupancy Map:** Reflects the current state of the environment with both static and dynamic objects.
- **Semantic 2D Map Integration:** Enhances environmental understanding using tools like GridMap.
- **Filtering and Fusion:** Applies filters to improve map accuracy.
- **ROS Integration:** Fully integrated with ROS.

Please check the subpackages README.md for more information

## Installation

Ensure ROS is installed. This package depends on several external ROS packages and libraries.

### Dependencies

1. **GridMap**
    - For semantic 2D map generation.
    ```bash
    git clone https://github.com/ANYbotics/grid_map.git
    ```
2. **Ground Plane Removal**
    - Essential for dynamic object segmentation.
    ```bash
    git clone git@github.com:leggedrobotics/tree_detection.git
    ```
3. **FKIE Message Filters**
    ```bash
    sudo apt install ros-noetic-fkie-message-filters
    ```

## Usage

1. Launch your sensors (camera and LiDAR) and publish the robot state.
2. Launch your segmentation node or use [rsl_panoptic](git@github.com:leggedrobotics/rsl_panoptic.git).
3. Launch the dynamic mapping node:
    ```bash
    roslaunch dynamic_mapping dynamic_mapping.launch
    ```
4. Launch the Panoptic Gridmap:
    ```bash
    roslaunch panoptic_gridmap semantic_map.launch
    ```