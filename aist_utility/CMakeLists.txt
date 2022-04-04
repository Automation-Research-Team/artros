cmake_minimum_required(VERSION 2.8.3)
project(aist_utility)

## Compile as C++17, supported in ROS melodic and newer
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS_RELEASE	"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")

find_package(PkgConfig)
pkg_check_modules(LIBTIFF REQUIRED libtiff-4)

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

#############
## Install ##
#############
install(
  DIRECTORY   include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
