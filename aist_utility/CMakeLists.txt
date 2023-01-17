cmake_minimum_required(VERSION 2.8.3)
project(aist_utility)

## Compile as C++17, supported in ROS melodic and newer
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE	"")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")
set(CMAKE_CXX_BUILD_TYPE	Release)

find_package(PkgConfig)
pkg_check_modules(LIBTIFF REQUIRED libtiff-4)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    genmsg
    message_generation
    sensor_msgs
    geometry_msgs
    tf
    ddynamic_reconfigure
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

###########
## Build ##
###########
add_message_files(
  FILES
    Flt.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
  INCLUDE_DIRS
    include
    ${OpenCV_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBTIFF_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  src
)

link_directories(
  ${LIBTIFF_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/tiff.cpp
  src/ply.cpp
  src/oply/OrderedPly.cpp
  src/rply/rply.c
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
  ${LIBTIFF_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(rotconv
  src/rotconv.cpp
)

target_link_libraries(rotconv
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_executable(butterworth_lpf_test
  src/butterworth_lpf_test.cpp
)

target_link_libraries(butterworth_lpf_test
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_dependencies(butterworth_lpf_test
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

add_executable(
  spline_interpolator_test
  src/spline_interpolator_test.cpp
)

target_link_libraries(
  spline_interpolator_test
  ${catkin_LIBRARIES}
)

add_dependencies(
  spline_interpolator_test
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############
install(
  TARGETS
    rotconv joint_state_extractor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY   include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
