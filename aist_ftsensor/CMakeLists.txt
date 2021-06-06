cmake_minimum_required(VERSION 2.8.3)
project(aist_ftsensor)

add_compile_options(-std=c++14)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML-CPP yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  std_msgs
  tf
)

# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aist_ftsensor
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/ftsensor.cpp src/main.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  yaml-cpp
)

add_library(${PROJECT_NAME}_nodelet src/ftsensor.cpp src/nodelet.cpp)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  yaml-cpp
)
