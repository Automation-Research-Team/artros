cmake_minimum_required(VERSION 3.20)
project(aist_handeye_calibration)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################
#  Dependencies  #
##################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(YAML-CPP REQUIRED yaml-cpp)

###########
#  Build  #
###########
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
    src/Calibrator.cpp
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
    ${PROJECT_NAME}::Calibrator
)

ament_auto_add_executable(
  test_calibrator
    src/test_calibrator.cpp
)

target_link_libraries(
  test_calibrator
    ${YAML-CPP_LIBRARIES}
)

########################
#  Python executables  #
########################
ament_python_install_package(${PROJECT_NAME})

# install(
#   PROGRAMS
#     scripts/run_calibration.py
#   DESTINATION
#     lib/${PROJECT_NAME}
# )

##########
#  Test  #
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

ament_auto_package(INSTALL_TO_SHARE launch config)
