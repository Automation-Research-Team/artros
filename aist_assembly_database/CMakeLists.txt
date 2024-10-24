cmake_minimum_required(VERSION 2.8.3)
project(aist_assembly_database)

find_package(catkin REQUIRED COMPONENTS
  rospy
  moveit_msgs
  visualization_msgs
  tf
  tf2_ros
)

catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

