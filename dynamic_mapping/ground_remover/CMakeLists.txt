cmake_minimum_required(VERSION 3.0.2)
project(ground_remover)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(PCL 1.10 REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    COMPONENTS
    ground_plane_removal
    tree_detection_ros
    pcl_conversions
    sensor_msgs
    roscpp
    roslib
    )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ground_remover
  CATKIN_DEPENDS ground_plane_removal tree_detection_ros
  DEPENDS PCL
)

###########
## Build ##
###########

include_directories(
     include
     ${PCL_INCLUDE_DIRS}
     ${catkin_INCLUDE_DIRS}

)
link_directories( #only needed for pcl
  ${PCL_LIBRARY_DIRS}
)

add_definitions( #only needed for pcl
  ${PCL_DEFINITIONS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/elevation_map_ground_remover.cpp
 )

add_executable(${PROJECT_NAME}_node src/ground_remover_node.cpp src/ground_remover_ros.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

# Add clang tooling
if(NOT DEFINED NO_CLANG_TOOLING)
  find_package(cmake_clang_tools QUIET)
  if(cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
    add_clang_tooling(
            TARGET ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_FIX
    )
  endif(cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
