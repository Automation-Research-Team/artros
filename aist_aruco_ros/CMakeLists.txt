cmake_minimum_required(VERSION 2.8.3)
project(aist_aruco_ros)

## Compile as C++11, supported in ROS Kinetic and newer
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS_RELEASE	"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")
set(CMAKE_CXX_BUILD_TYPE	Release)

find_package(catkin REQUIRED
  COMPONENTS
    ddynamic_reconfigure
    geometry_msgs
    visualization_msgs
    image_transport
    cv_bridge
    tf2_ros
    tf2_geometry_msgs
    aist_utility
    aruco
)

find_package(OpenCV REQUIRED)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_message_files(
  FILES
    ImagePoint.msg
    PointCorrespondence.msg
    PointCorrespondenceArray.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    geometry_msgs
    visualization_msgs
    pluginlib
    nodelet
)

add_library(aruco_detector
  src/Detector.cpp
)

add_dependencies(aruco_detector
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(aruco_detector
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(aruco_detector_2d
  src/Detector2D.cpp
)

add_dependencies(aruco_detector_2d
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(aruco_detector_2d
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(aruco_multi_detector_2d
  src/MultiDetector2D.cpp
)

add_dependencies(aruco_multi_detector_2d
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(aruco_multi_detector_2d
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    aruco_detector aruco_detector_2d
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
