cmake_minimum_required(VERSION 3.8)
project(aist_utility)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig)
pkg_check_modules(LIBTIFF REQUIRED libtiff-4)
pkg_check_modules(YAML-CPP yaml-cpp REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED 1.8 REQUIRED COMPONENTS common io)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(perception_pcl REQUIRED)
find_package(aist_msgs REQUIRED)
find_package(ddynamic_reconfigure2 REQUIRED)

########################################
#  shared library: libaist_utility.so  #
########################################
add_library(
  ${PROJECT_NAME}
    src/tiff.cpp
    src/ply.cpp
    src/oply/OrderedPly.cpp
    src/rply/rply.c
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(
  ${PROJECT_NAME} PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ${PROJECT_NAME}
    rclcpp_components
    sensor_msgs
    aist_msgs
)

target_link_libraries(
  ${PROJECT_NAME}
    ${LIBTIFF_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${YAML-CPP_LIBRARIES}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
#  rotconv  #
#############
add_executable(
  rotconv
    src/rotconv.cpp
)

##########################
#  butterworth_lpf_test  #
##########################
add_library(
  butterworth_lpf_test SHARED
    src/butterworth_lpf_test.cpp
)

target_include_directories(
  butterworth_lpf_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  butterworth_lpf_test
    rclcpp_components
    sensor_msgs
    aist_msgs
    ddynamic_reconfigure2
)

target_link_libraries(
  butterworth_lpf_test
    COMPOSITION_BUILDING_DLL
)

rclcpp_components_register_node(
  butterworth_lpf_test
    PLUGIN "${PROJECT_NAME}::ButterworthLPFTest"
    EXECUTABLE butterworth_lpf_test_node
)

###########################
#  joint_state_extractor  #
###########################
add_library(
  joint_state_extractor SHARED
    src/joint_state_extractor.cpp
)

target_include_directories(
  joint_state_extractor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  joint_state_extractor
    rclcpp_components
    sensor_msgs
    aist_msgs
    ddynamic_reconfigure2
)

target_link_libraries(
  joint_state_extractor
    COMPOSITION_BUILDING_DLL
)

rclcpp_components_register_node(
  joint_state_extractor
    PLUGIN "${PROJECT_NAME}::JointStateExtractor"
    EXECUTABLE joint_state_extractor_node
)

##############################
#  spline_extrapolator_test  #
##############################
#add_executable(
#  spline_extrapolator_test
#  src/spline_extrapolator_test.cpp
#)

#################
#  capture_pcd  #
#################
#add_executable(
#  capture_pcd
#    src/capture_pcd.cpp
#)

#target_link_libraries(
#  capture_pcd
#    ${OpenCV_LIBRARIES}
#)

#############
## Install ##
#############
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS  ${PROJECT_NAME} butterworth_lpf_test joint_state_extractor
  EXPORT   export_${PROJECT_NAME}
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)

# Ament exports
ament_export_libraries(${PROJECT_NAME})

ament_package()
