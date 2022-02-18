# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(aist_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  message_generation
  tf
  tf_conversions
  trajectory_msgs
  geometry_msgs
  control_msgs
  actionlib
  kdl_parser
)

find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
  ${Boost_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  include
)

add_action_files(FILES PoseHead.action)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    actionlib
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS trajectory_msgs control_msgs geometry_msgs
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

#add_executable(
#  pose_head_tracker
#  src/pose_head_tracker.cpp
#)

#target_link_libraries(
#  pose_head_tracker
#  ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES}
#)

#add_dependencies(
#  pose_head_tracker
#  ${catkin_EXPORTED_TARGETS}
#)

install(
  TARGETS point_head_tracker
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(
#  DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
