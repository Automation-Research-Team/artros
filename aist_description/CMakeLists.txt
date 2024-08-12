cmake_minimum_required(VERSION 2.8.3)
project(aist_description)

find_package(catkin REQUIRED COMPONENTS)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

#############
## Install ##
#############
catkin_install_python(
  PROGRAMS scripts/convert_stl_to_ply.py
           scripts/convert_o2ac_to_cop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
