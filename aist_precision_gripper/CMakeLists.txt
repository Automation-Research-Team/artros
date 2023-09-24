###############################################################################
# Set minimum required version of cmake, project name and compile options
###############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(aist_precision_gripper)

###############################################################################
# Find catkin packages and libraries for catkin and system dependencies
###############################################################################
find_package(
  catkin REQUIRED
    COMPONENTS
      rospy
      actionlib
      actionlib_msgs
      control_msgs
      sensor_msgs
      dynamixel_workbench_msgs
      dynamixel_workbench_toolbox
)

###############################################################################
# Declare catkin specific configuration to be passed to dependent projects
###############################################################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    actionlib_msgs
    control_msgs
    sensor_msgs
    dynamixel_workbench_msgs
    dynamixel_workbench_toolbox
)

catkin_install_python(
  PROGRAMS
    scripts/precision_gripper_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
