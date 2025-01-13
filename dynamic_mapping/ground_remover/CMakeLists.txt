cmake_minimum_required(VERSION 3.8)
project(ground_remover)

# Set C++ standard and compile options
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(ground_plane_removal REQUIRED)
#find_package(tree_detection_ros REQUIRED)

# Include directories
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_definitions(
    ${PCL_DEFINITIONS}
)

# Declare the library
add_library(${PROJECT_NAME} SHARED
    src/elevation_map_ground_remover.cpp
)

# Declare the executable
add_executable(${PROJECT_NAME}_node
    src/ground_remover_node.cpp
    src/ground_remover_ros.cpp
)

# Link libraries to the executable
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    
)


# Link dependencies to the library
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    pcl_conversions
    ground_plane_removal
    grid_map_pcl
    #tree_detection_ros
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    sensor_msgs
    pcl_conversions
    ground_plane_removal
    grid_map_pcl
    #tree_detection_ros
)

# Install the library
install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install include files
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

# Enable clang tooling
if(NOT DEFINED NO_CLANG_TOOLING)
    find_package(cmake_clang_tools QUIET)
    if(cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
        add_clang_tooling(
            TARGET ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_FIX
        )
    endif()
endif()

# Finalize the package
ament_package()
