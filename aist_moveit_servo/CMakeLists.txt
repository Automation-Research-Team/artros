# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(aist_moveit_servo)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")
set(CMAKE_CXX_BUILD_TYPE	Release)

# Load catkin and all dependencies required for this package
find_package(Boost REQUIRED COMPONENTS thread)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    tf_conversions
    tf2_geometry_msgs
    nav_msgs
    ddynamic_reconfigure
    trajectory_msgs
    control_msgs
    actionlib
    trac_ik_lib
    moveit_msgs
    moveit_ros_planning_interface
    rosparam_shortcuts
    control_toolbox
    aist_utility
    message_generation
)

catkin_python_setup()

add_action_files(
  FILES
    PoseTracking.action
)

add_message_files(
  FILES
    DurationArray.msg
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    actionlib
    trajectory_msgs
    control_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    trajectory_msgs
    control_msgs
    moveit_msgs
    moveit_ros_planning_interface
    rosparam_shortcuts
    control_toolbox
    message_runtime
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS}
)

#
#  libaist_moveit_servo
#
add_library(
  ${PROJECT_NAME}
  src/collision_check.cpp
  src/servo.cpp
  src/servo_parameters.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

#
#  servo_server
#
add_executable(
  servo_server
  src/servo_server.cpp
)

target_link_libraries(
  servo_server
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_dependencies(
  servo_server
  ${catkin_EXPORTED_TARGETS}
)

#
#  pose_tracking_servo
#
add_executable(
  pose_tracking_servo
  src/pose_tracking_servo.cpp
)

target_link_libraries(
  pose_tracking_servo
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_dependencies(
  pose_tracking_servo
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

#
#  velocity_feedforward_servo
#
add_executable(
  velocity_feedforward_servo
  src/velocity_feedforward_servo.cpp
)

target_link_libraries(
  velocity_feedforward_servo
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_dependencies(
  velocity_feedforward_servo
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

#
#  odometry_feedforward_servo
#
add_executable(
  odometry_feedforward_servo
  src/odometry_feedforward_servo.cpp
)

target_link_libraries(
  odometry_feedforward_servo
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_dependencies(
  odometry_feedforward_servo
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS servo_server pose_tracking_servo
	  velocity_feedforward_servo odometry_feedforward_servo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(
#  DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
