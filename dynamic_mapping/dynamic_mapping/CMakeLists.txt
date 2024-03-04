cmake_minimum_required(VERSION 3.0.2)
project(dynamic_mapping)

set(CATKIN_PACKAGE_DEPENDENCIES
        cv_bridge
        roscpp
        sensor_msgs
        std_msgs
        ground_remover
        jsk_recognition_msgs
        tf_conversions
        tf2
        tf2_sensor_msgs
        dynamic_mapping_msgs
        visualization_msgs
        open3d_catkin
        message_matching_msgs
        nodelet
        message_matching
        )

find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

find_package(Boost REQUIRED)
#find_package(Eigen3 3.4 REQUIRED)
#find_package(Open3D REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} yaml-cpp
        CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########
SET(EIGEN3_INCLUDE_DIRS "../thirdparty/eigen")

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS} # ORDER matters!
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/PointLabeler.cpp
        src/helpers.cpp
        src/MatchedMessageProcessor.cpp
        src/LidarCameraProjector.cpp
        src/tracking/Hungarian.cpp
        src/tracking/Track.cpp
        src/tracking/Tracker.cpp
        src/MovingObjectsFilter.cpp
        src/DynamicMappingRos.cpp
        src/dynamic_mapping_nodelet.cpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

add_executable(dynamic_mapping_node
        src/dynamic_mapping_node.cpp
        )

target_link_libraries(dynamic_mapping_node
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PROJECT_NAME}
        )


install(
        FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Add clang tooling
if (NOT DEFINED NO_CLANG_TOOLING)
    find_package(cmake_clang_tools QUIET)
    if (cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
        add_clang_tooling(
                TARGET dynamic_mapping_node
                SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
                CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
                CF_FIX
        )
    endif (cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
endif ()

