cmake_minimum_required(VERSION 2.8.3)
project(aist_handeye_calibration)

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
    roscpp
    image_proc
    image_transport
    std_srvs
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    tf2_ros
    tf2_geometry_msgs
    dynamic_reconfigure
    pcl_ros
    pcl_conversions
    genmsg
    actionlib
    actionlib_msgs
    aist_utility
)

generate_dynamic_reconfigure_options(
  cfg/aist_handeye_calibration.cfg
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
    std_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    geometry_msgs
    actionlib_msgs
)

## System dependencies are found with CMake's conventions

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

add_executable(calibrator
  src/main.cpp
  src/Calibrator.cpp
)

add_dependencies(calibrator
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(calibrator
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${YAML-CPP_LIBRARIES}
)

add_executable(test_calibrator
  src/test_calibrator/main.cpp
)

add_dependencies(test_calibrator
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(test_calibrator
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${YAML-CPP_LIBRARIES}
)

add_executable(plane_detector
  src/plane_detector/main.cpp
)

add_dependencies(plane_detector
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(plane_detector
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${pcl_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    calibrator test_calibrator plane_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    scripts/run_calibration.py
    scripts/check_calibration.py
    scripts/tool_calibration.py
    scripts/publish_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
