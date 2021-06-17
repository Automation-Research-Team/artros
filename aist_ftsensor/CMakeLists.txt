cmake_minimum_required(VERSION 2.8.3)
project(aist_ftsensor)

add_compile_options(-std=c++14)

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
)

# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    controller_interface
    geometry_msgs
    hardware_interface
    realtime_tools
    roscpp
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
              config/aist_ftsensor_controller.yaml
              launch/aist_ftsensor_controller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
