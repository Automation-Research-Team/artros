# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(aist_controllers)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")
set(CMAKE_CXX_BUILD_TYPE	Release)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  message_generation
  tf
  tf_conversions
  trajectory_msgs
  control_msgs
  actionlib
  kdl_parser
  trac_ik_lib
  moveit_servo
)

find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_python_setup()

include_directories(
  include
  ${Boost_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)

add_action_files(FILES PoseHead.action)

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
)

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

add_executable(
  pose_tracking_servo
  src/pose_tracking_servo.cpp
)

target_link_libraries(
  pose_tracking_servo
  ${Boost_LIBRARIES} ${catkin_LIBRARIES}
)

add_dependencies(
  pose_tracking_servo
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  pose_head_servo
  src/pose_head_servo.cpp
)

target_link_libraries(
  pose_head_servo
  ${Boost_LIBRARIES} ${catkin_LIBRARIES}
)

add_dependencies(
  pose_head_servo
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  pose_tracking_example
  src/moveit_servo/pose_tracking_example.cpp
  src/moveit_servo/collision_check.cpp
  src/moveit_servo/low_pass_filter.cpp
  src/moveit_servo/pose_tracking.cpp
  src/moveit_servo/servo.cpp
  src/moveit_servo/servo_calcs.cpp
)

target_link_libraries(
  pose_tracking_example
  ${Boost_LIBRARIES} ${catkin_LIBRARIES}
)

add_dependencies(
  pose_tracking_example
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS point_head_tracker pose_head_tracker pose_group_tracker
  pose_tracking_servo pose_head_servo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(
#  DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
