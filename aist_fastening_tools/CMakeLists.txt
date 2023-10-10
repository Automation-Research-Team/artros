##########################################################################
# Set minimum required version of cmake, project name and compile options
##########################################################################
cmake_minimum_required(VERSION 2.8.3)
project(aist_fastening_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++17)
add_compile_options(-fpermissive)
add_compile_options(-pthread)
add_compile_options(-O2)

##########################################################################
# Find catkin packages and libraries for catkin and system dependencies
##########################################################################
find_package(
  catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  dynamixel_workbench_controllers
  dynamixel_workbench_msgs
  ddynamic_reconfigure
  message_generation
  actionlib
  actionlib_msgs
  aist_utility
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
)

##########################################################################
# Declare catkin specific configuration to be passed to dependent projects
##########################################################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    dynamixel_workbench_controllers
    dynamixel_workbench_msgs
    ddynamic_reconfigure
    message_runtime
    actionlib_msgs
)

##########################################################################
# Build
##########################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(screw_tool_controller
  src/screw_tool_controller.cpp
)

add_dependencies(screw_tool_controller
  ${PROJECT_NAME}_gencpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(screw_tool_controller
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
)

##########################################################################
# Install
##########################################################################
install(
  TARGETS
    screw_tool_controller
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    scripts/screw_tool_test.py
    scripts/suction_tool_controller.py
    scripts/suction_tool_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
