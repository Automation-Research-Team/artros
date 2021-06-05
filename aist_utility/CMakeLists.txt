cmake_minimum_required(VERSION 2.8.3)
project(aist_utility)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

add_library(
  ${PROJECT_NAME}
  src/aist_utility.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

#############
## Install ##
#############
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
