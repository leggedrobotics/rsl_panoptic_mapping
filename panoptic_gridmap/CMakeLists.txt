cmake_minimum_required(VERSION 3.5.1)
project(panoptic_gridmap)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        grid_map_core
        grid_map_ros
        grid_map_cv
        grid_map_filters
        grid_map_loader
        grid_map_msgs
        grid_map_octomap
        grid_map_rviz_plugin
        grid_map_visualization
        pcl_ros
        tf
        )


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#add_service_files(
#        DIRECTORY srv
#)

generate_messages(
        DEPENDENCIES
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(panoptic_gridmap
        src/SemanticMap.cpp
        src/semantic_map_node.cpp
        )

## Specify libraries to link a library or executable target against
target_link_libraries(
        panoptic_gridmap
        ${catkin_LIBRARIES} ${PCL_LIBRARIES}
        yaml-cpp
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS
        panoptic_gridmap
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
        DIRECTORY config data doc launch rviz scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    add_default_clang_tooling(
            DISABLE_CLANG_FORMAT
    )
endif (cmake_clang_tools_FOUND)