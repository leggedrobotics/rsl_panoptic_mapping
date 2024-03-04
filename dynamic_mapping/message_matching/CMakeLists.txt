cmake_minimum_required(VERSION 3.0.2)
project(message_matching)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        sensor_msgs
        fkie_message_filters
        message_matching_msgs
        )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES message_matching
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/message_matching_nodelet.cpp
   src/MessageMatcher.cpp
 )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

install(
        FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)