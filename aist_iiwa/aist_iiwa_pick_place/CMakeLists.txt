cmake_minimum_required(VERSION 2.8.3)
project(aist_iiwa_pick_place)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(
  PROGRAMS scripts/pose.py scripts/pick.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
