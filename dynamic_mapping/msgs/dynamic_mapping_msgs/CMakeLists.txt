cmake_minimum_required(VERSION 3.5)
project(dynamic_mapping_msgs)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
	"msg/Track.msg"
	"msg/TrackArray.msg"
	)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	DEPENDENCIES
	builtin_interfaces
	std_msgs
	geometry_msgs
	)
ament_export_dependencies(rosidl_default_runtime)
ament_package()


