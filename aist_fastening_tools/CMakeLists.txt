##########################################################################
# Set minimum required version of cmake, project name and compile options
##########################################################################
cmake_minimum_required(VERSION 2.8.3)
project(aist_fastening_tools)

##########################################################################
# Find catkin packages and libraries for catkin and system dependencies
##########################################################################
find_package(
  catkin REQUIRED COMPONENTS
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  message_generation
  genmsg
  actionlib
  actionlib_msgs
  rospy
)

##########################################################################
# Declare ROS messages, services and actions
##########################################################################

add_action_files(
  FILES
    ScrewToolCommand.action
    SuctionToolCommand.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

##########################################################################
# Declare catkin specific configuration to be passed to dependent projects
##########################################################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    dynamixel_workbench_msgs
    dynamixel_workbench_toolbox
    message_runtime
    actionlib_msgs
)

##########################################################################
# Build
##########################################################################

##########################################################################
# Install
##########################################################################
catkin_install_python(
  PROGRAMS
    scripts/motor_id_setup.py
    scripts/screw_tool_controller.py
    scripts/screw_tool_test.py
    scripts/suction_tool_controller.py
    scripts/suction_tool_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
