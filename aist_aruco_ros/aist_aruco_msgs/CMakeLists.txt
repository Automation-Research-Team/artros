cmake_minimum_required(VERSION 3.8)
project(aist_aruco_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(
  msg_files
    msg/PointCorrespondence.msg
    msg/PointCorrespondenceArray.msg
    msg/PointCorrespondenceArrayArray.msg
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    ${msg_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
