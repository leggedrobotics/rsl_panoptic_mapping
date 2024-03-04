# RSL Panoptic Mapping

![oveview.png](docs%2Foveview.png)
RSL Panoptic Mapping is a comprehensive ROS package designed for advanced environment understanding and navigation in dynamic settings. It leverages the power of semantic segmentation and dynamic object tracking to create detailed panoptic maps that distinguish between movable and static objects. This enables robotic systems to navigate complex environments more effectively.
## Features

- **Semantic Layer Generation:** Utilizes both dynamic and static point clouds to generate semantic layers that include detailed information about movable and static objects in the environment.
- **Dynamic Mapping and Tracking:** Processes raw LiDAR scans to separate point clouds into labeled groups, distinguishing between static and dynamic objects for real-time tracking and mapping.
- **Traversability Cost Layer:** Assigns navigation costs based on semantic information, aiding in the planning of efficient and safe routes through varying terrains.
- **Occupancy Map:** An advanced occupancy grid that reflects the current state of the environment, incorporating both static and dynamic objects.
- **Semantic 2D Map Integration:** Combines the capabilities of semantic layering and dynamic mapping to enhance environmental understanding, using tools like GridMap for high-level navigation planning.
- **Filtering and Fusion:** Applies median filters and other fusion techniques to improve map accuracy and reliability.
- **ROS Integration:** Fully integrated with the Robot Operating System (ROS) for seamless deployment and operation within robotic applications.

## Installation

To use RSL Panoptic Mapping, ensure you have ROS installed on your system. This package depends on several external ROS packages and libraries.

### Dependencies

1. **GridMap**
    - For semantic 2D map generation and handling.
   ```bash
   git clone https://github.com/ANYbotics/grid_map.git
   ```
Ground Plane Removal (Essential for dynamic object segmentation and mapping).
```bash
git clone git@github.com:leggedrobotics/tree_detection.git
```
FKIE Message Filters
```bash
    sudo apt install ros-noetic-fkie-message-filters
```

## Usage
Launch your sensors (camera and LiDAR) and publish the state of the robot. 
Launch your segmentation node or use [rsl_panoptic](git@github.com:leggedrobotics/rsl_panoptic.git).
Launch the dynamic mapping node:
```
roslaunch dynamic_mapping dynamic_mapping.launch
```
Launch the Panoptic Gridmap:
```
roslaunch panoptic_gridmap semantic_map.launch
```

