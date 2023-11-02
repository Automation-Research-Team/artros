cmake_minimum_required(VERSION 2.8.3)
project(aist_visualization)

add_compile_options(-std=c++14)
add_compile_options(-D _CODE)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation
    tf2_ros
    tf2_geometry_msgs
    shape_msgs
    rviz
)

find_package(OpenCV)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

add_message_files(
  FILES
    TexturedMeshStamped.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    shape_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    pluginlib
    nodelet
    tf2_ros
    tf2_geometry_msgs
    shape_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${rviz_INCLUDE_DIRS}
)

message("-- OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

add_executable(mesh_generator
  src/mesh_generator.cpp
)

add_dependencies(mesh_generator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mesh_generator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LEAG_LIBS}
)

add_library(textured_mesh_display
  src/textured_mesh_display.cpp
)

add_dependencies(textured_mesh_display
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(textured_mesh_display
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${QT_LIBRARIES}
)

install(
  TARGETS
    mesh_generator textured_mesh_display
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
