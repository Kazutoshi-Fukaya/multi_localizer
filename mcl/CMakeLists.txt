cmake_minimum_required(VERSION 3.0.2)
project(mcl)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  multi_localizer_msgs
  nav_msgs
  object_detector_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# mcl_node
add_executable(mcl_node
  mcl_base/mcl_base.cpp
  mcl_base/particle.cpp
  src/mcl.cpp
  src/mcl_node.cpp
)
target_link_libraries(mcl_node ${catkin_LIBRARIES})