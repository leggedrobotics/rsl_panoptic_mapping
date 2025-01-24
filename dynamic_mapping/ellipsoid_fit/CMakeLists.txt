cmake_minimum_required(VERSION 3.5)
project(ellipsoid_fit)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

# Include directories
include_directories(include)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# Add the executable
add_executable(ellipsoid_fit_node src/ellipsoid_fit_node.cpp src/ellipsoid_fit.cpp)




# Link libraries
ament_target_dependencies(ellipsoid_fit_node rclcpp sensor_msgs tf2_ros tf2_geometry_msgs tf2_sensor_msgs PCL pcl_conversions pcl_msgs)
target_link_libraries(ellipsoid_fit_node
  ${PCL_LIBRARIES}
)
# Install the executable
install(TARGETS ellipsoid_fit_node
  DESTINATION lib/${PROJECT_NAME})

# Install include directories
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()