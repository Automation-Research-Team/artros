cmake_minimum_required(VERSION 3.8)
project(aist_description)

find_package(ament_cmake_core REQUIRED)

#############
## Install ##
#############
install(
  PROGRAMS
    scripts/convert_stl_to_ply.py
    scripts/generate_urdf_from_meshes.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY devices
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY objects
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY parts
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY scenes
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY wrc
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
