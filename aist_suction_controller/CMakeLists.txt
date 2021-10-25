###############################################################################
# Set minimum required version of cmake, project name and compile options
###############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(aist_suction_controller)

###############################################################################
# Find catkin packages and libraries for catkin and system dependencies
###############################################################################
find_package(
  catkin REQUIRED
  COMPONENTS
    message_generation
    genmsg
    actionlib
    actionlib_msgs
    std_msgs
)

###############################################################################
# Declare ROS messages, services and actions
###############################################################################
add_action_files(
  FILES
    SuctionControl.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

###############################################################################
# Declare catkin specific configuration to be passed to dependent projects
###############################################################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    actionlib_msgs
    std_msgs
)
