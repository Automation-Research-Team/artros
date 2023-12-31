cmake_minimum_required(VERSION 2.8.3)
project(aist_routines)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    cmake_modules
    geometry_msgs
    std_srvs
    std_msgs
    sensor_msgs
    visualization_msgs
    actionlib_msgs
)

catkin_python_setup()

# Declare the message files to be built
#add_message_files(DIRECTORY msg)

#add_service_files(DIRECTORY srv)

# Generate actions in the 'action' folder
add_action_files(DIRECTORY action)

# # Generate added messages and services with any dependencies listed here

# Actually generate the language-specific message and service files
generate_messages(
  DEPENDENCIES
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  actionlib_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/interactive.py
    scripts/kitting.py
    scripts/hmi_server.py
    scripts/hmi_demo.py
    scripts/iiwa_interactive.py
    scripts/base_interactive.py
    scripts/fetch_interactive.py
    scripts/test.py
    scripts/test_wrist_cameras.py
    scripts/ros2nep.py
    scripts/camera_script.py
    scripts/getPointing.py
    scripts/setRequestHelp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
