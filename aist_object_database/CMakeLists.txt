cmake_minimum_required(VERSION 2.8.3)
project(aist_object_database)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    std_msgs
    geometry_msgs
    shape_msgs
    aist_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  shape_msgs
  aist_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/object_database_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
