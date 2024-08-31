cmake_minimum_required(VERSION 2.8.3)
project(aist_collision_object_manager)

find_package(
  catkin REQUIRED
  COMPONENTS
    cmake_modules
    std_msgs
    geometry_msgs
    shape_msgs
    visualization_msgs
    moveit_msgs
    aist_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    shape_msgs
    visualization_msgs
    moveit_msgs
    aist_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/collision_object_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
