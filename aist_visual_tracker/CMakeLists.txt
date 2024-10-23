cmake_minimum_required(VERSION 2.8.3)
project(aist_visual_tracker)

## Compile as C++17, supported in ROS melodic and newer
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG -mavx2 -DAVX2 -Wunknown-pragmas")
set(CMAKE_CXX_FLAGS_RELEASE	"")
set(CMAKE_CXX_FLAGS_DEBUG	"-g -DPROFILE")
set(CMAKE_CXX_BUILD_TYPE	Release)

find_package(
  catkin REQUIRED
  COMPONENTS
    ddynamic_reconfigure
    sensor_msgs
    image_transport
    cv_bridge
    rospy
)

find_package(OpenCV REQUIRED)

#catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  src
)

#add_message_files(FILES FileInfo.msg)

#generate_messages(DEPENDENCIES sensor_msgs geometry_msgs)

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
  src/KLTTracker.cpp
  src/klt/convolve.c
  src/klt/error.c
  src/klt/klt.c
  src/klt/klt_util.c
  src/klt/pnmio.c
  src/klt/pyramid.c
  src/klt/selectGoodFeatures.c
  src/klt/storeFeatures.c
  src/klt/trackFeatures.c
  src/klt/writeFeatures.c
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
