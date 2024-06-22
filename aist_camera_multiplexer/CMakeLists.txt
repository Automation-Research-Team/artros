cmake_minimum_required(VERSION 3.8)
project(aist_camera_multiplexer)

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
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(ddynamic_reconfigure2 REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

###################################################
#  shared library: libaist_camera_multiplexer.so  #
###################################################
add_library(
  ${PROJECT_NAME} SHARED
    src/Multiplexer.cpp
)

target_compile_definitions(
  ${PROJECT_NAME} PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ${PROJECT_NAME}
    rclcpp_components
    sensor_msgs
    image_transport
    ddynamic_reconfigure2
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
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
  PROGRAMS
    scripts/drealsense.py
    scripts/select_camera.py
    scripts/select_realsense_camera.py
  DESTINATION
    lib/${PROJECT_NAME}
)

#############
## Exports ##
#############
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(image_transport)

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
