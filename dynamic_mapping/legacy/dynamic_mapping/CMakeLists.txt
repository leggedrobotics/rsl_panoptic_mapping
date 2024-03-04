cmake_minimum_required(VERSION 3.0.2)
project(dynamic_mapping_py)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_mapping
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories()

catkin_python_setup()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  dynamic_mapping_py/dynamic_mapping_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
