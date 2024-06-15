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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(aist_msgs REQUIRED)
find_package(ddynamic_reconfigure2 REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

########################################
#  shared library: libaist_utility.so  #
########################################
add_library(
  ${PROJECT_NAME} SHARED
    src/tiff.cpp
    src/ply.cpp
    src/oply/OrderedPly.cpp
    src/rply/rply.c
)

target_compile_definitions(
  ${PROJECT_NAME} PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ${PROJECT_NAME}
    rclcpp_components
    sensor_msgs
    tf2
)

target_link_libraries(
  ${PROJECT_NAME}
    ${LIBTIFF_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${YAML-CPP_LIBRARIES}
)

#############
#  rotconv  #
#############
add_executable(
  rotconv
    src/rotconv.cpp
)

ament_target_dependencies(
  rotconv
    tf2
)

###########################
#  joint_state_extractor  #
###########################
add_library(
  joint_state_extractor SHARED
    src/joint_state_extractor.cpp
)

target_compile_definitions(
  joint_state_extractor PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  joint_state_extractor
    rclcpp_components
    sensor_msgs
    ddynamic_reconfigure2
)

rclcpp_components_register_node(
  joint_state_extractor
    PLUGIN "${PROJECT_NAME}::JointStateExtractor"
    EXECUTABLE joint_state_extractor_node
)

#################
#  capture_pcd  #
#################
add_library(
  capture_pcd SHARED
    src/capture_pcd.cpp
)

target_compile_definitions(
  capture_pcd PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  capture_pcd
    rclcpp_components
    sensor_msgs
    std_srvs
    image_transport
    pcl_ros
    pcl_conversions
    tf2
    tf2_ros
    ddynamic_reconfigure2
)

target_link_libraries(
  capture_pcd
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

rclcpp_components_register_node(
  capture_pcd
    PLUGIN "${PROJECT_NAME}::PCDCapturer"
    EXECUTABLE capture_pcd_node
)

##########################
#  butterworth_lpf_test  #
##########################
add_library(
  butterworth_lpf_test SHARED
    src/butterworth_lpf_test.cpp
)

target_compile_definitions(
  butterworth_lpf_test PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  butterworth_lpf_test
    rclcpp_components
    sensor_msgs
    ddynamic_reconfigure2
    aist_msgs
)

rclcpp_components_register_node(
  butterworth_lpf_test
    PLUGIN "${PROJECT_NAME}::ButterworthLPFTest"
    EXECUTABLE butterworth_lpf_test_node
)

##############################
#  spline_extrapolator_test  #
##############################
add_library(
  spline_extrapolator_test SHARED
    src/spline_extrapolator_test.cpp
)

target_compile_definitions(
  spline_extrapolator_test PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  spline_extrapolator_test
    rclcpp_components
    sensor_msgs
    ddynamic_reconfigure2
    aist_msgs
)

rclcpp_components_register_node(
  spline_extrapolator_test
    PLUGIN "${PROJECT_NAME}::SplineExtrapolatorTest"
    EXECUTABLE spline_extrapolator_test_node
)

#############
## Install ##
#############
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME} joint_state_extractor capture_pcd
    butterworth_lpf_test spline_extrapolator_test
  EXPORT   export_${PROJECT_NAME}
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

#########################
## Install python code ##
#########################
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/gensig.py
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Exports ##
#############
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(aist_msgs std_srvs pcl_ros pcl_conversions)

##########
#  test  #
##########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
