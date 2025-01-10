cmake_minimum_required(VERSION 3.8)
project(ground_plane_removal)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ----------------------
# Sources
# ----------------------
set(SRC_FILES
  src/GroundPlaneRemover.cpp
  src/Parameters.cpp
  src/ElevationMapGroundPlaneRemover.cpp
)

# ----------------------
# Dependencies
# ----------------------
find_package(ament_cmake REQUIRED)

# If you need grid_map_pcl in ROS 2
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
# For PCL
find_package(PCL 1.10 REQUIRED)
find_package(rclcpp REQUIRED)
# For yaml-cpp (if you need it)
find_package(yaml-cpp REQUIRED)

# For OpenMP
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-DGROUND_PLANE_REMOVAL_OPENMP_FOUND=${OPENMP_FOUND})
endif()

# ----------------------
# Include & Link
# ----------------------
# Include directories
include_directories(
  include
  SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${OpenMP_CXX_INCLUDE_DIRS}
)

link_directories( # only needed for PCL
  ${PCL_LIBRARY_DIRS}
)

add_definitions(  # only needed for PCL
  ${PCL_DEFINITIONS}
)

# ----------------------
# Build the library
# ----------------------
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

# Instead of catkin_package(), use ament commands
# Link + register ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  "grid_map_pcl"
  "grid_map_msgs"
  "grid_map_ros"
  "grid_map_core"
  "yaml-cpp"
  "rclcpp"
)

# Link raw libraries (PCL, OpenMP, etc.)
target_link_libraries(${PROJECT_NAME}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
  yaml-cpp
)

# ----------------------
# Install
# ----------------------
# Install the library itself
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# If you have launch, param, or other resources,
# install them similarly, e.g.:
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# ----------------------
# Export package info
# ----------------------
# Make this library available to other packages
#ament_export_dependencies(grid_map_pcl)
#ament_export_dependencies(PCL)
#ament_export_dependencies(yaml-cpp)
#ament_export_interfaces(${PROJECT_NAME})
#ament_export_libraries(${PROJECT_NAME})

# Finally...
ament_package()
