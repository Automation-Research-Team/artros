cmake_minimum_required(VERSION 3.5.0)
project(aist_aruco_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    cmake_modules
    geometry_msgs
    std_msgs
)

# Declare the message files to be built
add_message_files(DIRECTORY msg)

# # Generate added messages and services with any dependencies listed here

# Actually generate the language-specific message and service files
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)
