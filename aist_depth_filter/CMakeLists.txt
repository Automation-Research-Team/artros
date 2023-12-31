cmake_minimum_required(VERSION 2.8.3)
project(aist_depth_filter)

## Compile as C++17, supported in ROS melodic and newer
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")
set(CMAKE_CXX_BUILD_TYPE	Release)

find_package(PkgConfig)
pkg_check_modules(LIBTIFF REQUIRED libtiff-4)

find_package(
  catkin REQUIRED
  COMPONENTS
    ddynamic_reconfigure
    sensor_msgs
    image_transport
    message_generation
    actionlib_msgs
    actionlib
    rospy
    aist_utility
)

find_package(OpenCV REQUIRED)

catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBTIFF_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_message_files(FILES FileInfo.msg Plane.msg PlaneStamped.msg)
add_action_files(FILES DetectPlane.action)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    actionlib
    sensor_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    message_runtime
    sensor_msgs
    pluginlib
    nodelet
)

add_library(
  ${PROJECT_NAME}
  src/nodelet.cpp
  src/DepthFilter.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${LIBTIFF_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/image_feeder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
