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
    tf_conversions
    tf2_geometry_msgs
    ddynamic_reconfigure
    trajectory_msgs
    control_msgs
    actionlib
    kdl_parser
    trac_ik_lib
    nlopt
    control_toolbox
    aist_utility
    message_generation
)

catkin_python_setup()

add_action_files(
  FILES
    PoseHead.action
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

install(
  TARGETS point_head_tracker pose_head_tracker pose_group_tracker
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    scripts/pose_head_client.py
    scripts/btlpf.py
    scripts/point_head_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(
#  DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
