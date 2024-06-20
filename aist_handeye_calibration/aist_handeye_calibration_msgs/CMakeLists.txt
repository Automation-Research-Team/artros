cmake_minimum_required(VERSION 3.8)
project(aist_handeye_calibration_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(
  srv_files
    srv/ComputeCalibration.srv
    srv/GetSampleList.srv
)

set(
  action_files
    action/TakeSample.action
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    ${srv_files}
    ${action_files}
  DEPENDENCIES
    geometry_msgs
    action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
