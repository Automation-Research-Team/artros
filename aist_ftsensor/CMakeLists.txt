cmake_minimum_required(VERSION 2.8.3)
project(aist_ftsensor)

## Compile as C++17, supported in ROS melodic and newer
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS_RELEASE	"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML-CPP yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  roslib
  std_msgs
  tf
  aist_utility
)

# catkin_python_setup()

add_service_files(
  FILES
    SetDecay.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    controller_interface
    geometry_msgs
    hardware_interface
    realtime_tools
    message_runtime
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_controller
  src/aist_ftsensor_controller.cpp
)

add_dependencies(
  ${PROJECT_NAME}_controller
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_controller
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  yaml-cpp
)

add_executable(ft300_driver src/ft300_driver.cpp)

target_link_libraries(ft300_driver
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_executable(dynpick_driver src/dynpick_driver.cpp)

target_link_libraries(dynpick_driver
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

#############
## Install ##
#############
# Install targets
install(TARGETS ${PROJECT_NAME}_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Intall plugins
install(FILES aist_ftsensor_plugin.xml
              config/ftsensor_controller.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
