cmake_minimum_required(VERSION 3.8)
project(message_matching)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(message_matching_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add the library
add_library(${PROJECT_NAME} SHARED
  src/message_matching_node.cpp
  src/MessageMatcher.cpp
)
add_executable(message_matching_node src/message_matching_node.cpp)

target_link_libraries(message_matching_node
    message_matching
    ${rclcpp_LIBRARIES}
)
#target_link_libraries(${PROJECT_NAME}
#  rclcpp::rclcpp
#  #sensor_msgs::sensor_msgs
#  #message_filters::message_filters
#  #message_matching_msgs::message_matching_msgs
#)
# Link the library against the packages you found:
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  message_matching_msgs
  message_filters
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  message_matching_msgs
  message_filters
)



# Install the library or executable targets
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
    message_matching_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install configuration files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install package information (like package.xml)
#install(
#  DIRECTORY share/${PROJECT_NAME}
#  DESTINATION share/${PROJECT_NAME}
#)

# Export package
ament_package()
