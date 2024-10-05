cmake_minimum_required(VERSION 3.8)
project(aist_robotiq_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

set(
  msg_files
    msg/CModelCommand.msg
    msg/CModelStatus.msg
    msg/EPickCommand.msg
)

set(
  srv_files
    srv/SetVelocity.srv
)

set(
  action_files
    action/EPickCommand.action
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
