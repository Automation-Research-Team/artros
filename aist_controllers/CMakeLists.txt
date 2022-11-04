# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(aist_controllers)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG -DBUTTERWORTH")
#set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")
set(CMAKE_CXX_BUILD_TYPE	Release)

set(SERVO_LIB_NAME moveit_servo_cpp_api)

# Load catkin and all dependencies required for this package
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    genmsg
    message_generation
    tf_conversions
    tf2_geometry_msgs
    ddynamic_reconfigure
    trajectory_msgs
    control_msgs
    actionlib
    kdl_parser
    trac_ik_lib
    nlopt
    moveit_msgs
    moveit_ros_planning_interface
    rosparam_shortcuts
    control_toolbox
    aist_utility
)

catkin_python_setup()

add_action_files(
  FILES
    PoseHead.action
    PoseTracking.action
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
    message_runtime
    trajectory_msgs
    control_msgs
    moveit_msgs
    moveit_ros_planning_interface
    rosparam_shortcuts
    control_toolbox
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)

#
#  point_head_tracker
#
add_executable(
  point_head_tracker
  src/point_head_tracker.cpp
)

target_link_libraries(
  point_head_tracker
  ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES}
)

add_dependencies(
  point_head_tracker
  ${catkin_EXPORTED_TARGETS}
)

#
#  pose_head_tracker
#
add_executable(
  pose_head_tracker
  src/pose_head_tracker.cpp
)

target_link_libraries(
  pose_head_tracker
  ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES}
)

add_dependencies(
  pose_head_tracker
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

#
#  pose_group_tracker
#
add_executable(
  pose_group_tracker
  src/pose_group_tracker.cpp
)

target_link_libraries(
  pose_group_tracker
  ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES}
)

add_dependencies(
  pose_group_tracker
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

#
#  libmoveit_servo_cpp_api
#
add_library(
  ${SERVO_LIB_NAME}
  src/moveit_servo/collision_check.cpp
  src/moveit_servo/low_pass_filter.cpp
  src/moveit_servo/servo.cpp
  src/moveit_servo/servo_calcs.cpp
)

target_link_libraries(
  ${SERVO_LIB_NAME}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(
  ${SERVO_LIB_NAME}
  ${catkin_EXPORTED_TARGETS}
)

#
#  libpose_tracking
#
add_library(
  pose_tracking
  src/moveit_servo/pose_tracking.cpp
)

target_link_libraries(
  pose_tracking
  ${catkin_LIBRARIES}
  ${SERVO_LIB_NAME}
)

add_dependencies(
  pose_tracking
  ${catkin_EXPORTED_TARGETS}
)

#
#  servo_server
#
add_executable(
  servo_server
  src/moveit_servo/servo_server.cpp
)

target_link_libraries(
  servo_server
  ${catkin_LIBRARIES}
  ${SERVO_LIB_NAME}
)

add_dependencies(
  servo_server
  ${catkin_EXPORTED_TARGETS}
)

#
#  pose_tracking_example
#
add_executable(
  pose_tracking_example
  src/moveit_servo/pose_tracking_example.cpp
)

target_link_libraries(
  pose_tracking_example
  pose_tracking
)

add_dependencies(
  pose_tracking_example
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  pose_tracking_servo
  src/pose_tracking_servo.cpp
)

target_link_libraries(
  pose_tracking_servo
  ${catkin_LIBRARIES}
  ${SERVO_LIB_NAME}
)

add_dependencies(
  pose_tracking_servo
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

#
#  joint_state_extractor
#
add_executable(
  joint_state_extractor
  src/joint_state_extractor.cpp
)

target_link_libraries(
  joint_state_extractor
  ${catkin_LIBRARIES}
)

add_dependencies(
  joint_state_extractor
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS point_head_tracker pose_head_tracker pose_group_tracker servo_server
  pose_tracking_servo joint_state_extractor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(
#  DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
