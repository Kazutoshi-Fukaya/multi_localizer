cmake_minimum_required(VERSION 3.0.2)
project(relative_recognition_localizer)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  multi_localizer_msgs
  nav_msgs
  object_color_detector_msgs
  roscpp
  rospy
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

# relative_recognition_localizer_node
add_executable(${PROJECT_NAME}_node
    src/particle.cpp
    src/mcl.cpp
    src/mcl_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
