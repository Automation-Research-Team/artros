cmake_minimum_required(VERSION 3.5)
project(aist_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

set(
  msg_files
    msg/DisplayItem.msg
    msg/Float32Stamped.msg
    msg/Pointing.msg
    msg/RequestHelp.msg
    msg/UpsetResult.msg
)

set(
  action_files
    action/AttemptBin.action
    action/PickOrPlace.action
    action/RepeatSweep.action
    action/RequestHelp.action
    action/Sweep.action
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    ${msg_files}
    ${action_files}
  DEPENDENCIES
    std_srvs
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    action_msgs
    moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)
