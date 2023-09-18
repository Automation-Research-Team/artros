cmake_minimum_required(VERSION 2.8.3)
project(aist_camera_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE	"")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")
set(CMAKE_CXX_BUILD_TYPE	Release)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML-CPP yaml-cpp REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    std_srvs
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
    dynamic_reconfigure
    actionlib
    actionlib_msgs
    aist_utility
    aist_aruco_ros
)

add_service_files(
  FILES
    GetSampleList.srv
    ComputeCalibration.srv
)

add_action_files(
  FILES
    TakeSample.action
)

generate_messages(
  DEPENDENCIES
    std_srvs
    std_msgs
    geometry_msgs
    sensor_msgs
    actionlib_msgs
    aist_aruco_ros
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    actionlib_msgs
    aist_aruco_ros
    pluginlib
    nodelet
)

## System dependencies are found with CMake's conventions

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(camera_calibrator
  src/Calibrator.cpp
)

add_dependencies(camera_calibrator
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(camera_calibrator
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${YAML-CPP_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    camera_calibrator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#catkin_install_python(
#  PROGRAMS
#    scripts/run_calibration.py
#    scripts/check_calibration.py
#    scripts/tool_calibration.py
#    scripts/publish_calibration.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
