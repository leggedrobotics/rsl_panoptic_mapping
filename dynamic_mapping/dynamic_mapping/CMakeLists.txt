cmake_minimum_required(VERSION 3.8)
project(dynamic_mapping)


# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Open3D REQUIRED)
find_package(dynamic_mapping_msgs REQUIRED)
find_package(message_matching_msgs REQUIRED)
find_package(message_matching REQUIRED)
find_package(ground_remover REQUIRED)
find_package(jsk_recognition_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(ground_plane_removal REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)

SET(EIGEN3_INCLUDE_DIRS "../thirdparty/eigen")
# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# Declare the library
add_library(${PROJECT_NAME} SHARED
  src/PointLabeler.cpp
  src/helpers.cpp
  src/MatchedMessageProcessor.cpp
  src/LidarCameraProjector.cpp
  src/tracking/Hungarian.cpp
  src/tracking/Track.cpp
  src/tracking/Tracker.cpp
  src/MovingObjectsFilter.cpp
  src/DynamicMappingRos.cpp
)

# Declare the executable
add_executable(dynamic_mapping_node
  src/dynamic_mapping_node.cpp
)

#target_link_libraries(${PROJECT_NAME}        # i.e. "dynamic_mapping"
#    
#    ${Boost_LIBRARIES}
#    ${PCL_LIBRARIES}
#)


target_link_libraries(dynamic_mapping_node
  #ground_remover
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  tf2
  tf2_eigen
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  Open3D
  dynamic_mapping_msgs
  message_matching_msgs
  message_matching
  jsk_recognition_msgs
  ground_plane_removal
  ground_remover
  grid_map_pcl
  pcl_conversions
)



ament_target_dependencies(dynamic_mapping_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  Open3D
  dynamic_mapping_msgs
  message_matching_msgs
  message_matching
  ground_remover
  jsk_recognition_msgs
  ground_plane_removal
  grid_map_pcl
  pcl_conversions
  tf2_eigen
)

install(TARGETS
    dynamic_mapping_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS ${PROJECT_NAME} dynamic_mapping_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# Install configuration files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)



# Install other resources
#install(
#  FILES nodelet_plugins.xml
#  DESTINATION share/${PROJECT_NAME}
#)


ament_package()