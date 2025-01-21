cmake_minimum_required(VERSION 3.8)
project(message_matching_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

################################################
## Declare ROS messages, services, and actions ##
################################################

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MatchedPair.msg"  # Add your message file here
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)

###################################
## Install and export settings ##
###################################

# Install the package
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
