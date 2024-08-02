cmake_minimum_required(VERSION 2.8.3)
project(aist_routines)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    geometry_msgs
    std_srvs
    std_msgs
    sensor_msgs
    visualization_msgs
    actionlib_msgs
)

catkin_python_setup()

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
    scripts/run_kitting.py
    scripts/run_assembly.py
    scripts/run_hmi_demo.py
    scripts/hmi_server.py
    scripts/interactive.py
    scripts/iiwa_interactive.py
    scripts/base_interactive.py
    scripts/fetch_interactive.py
    scripts/test_wrist_cameras.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
