cmake_minimum_required(VERSION 3.0.2)
project(object_localizer)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
   geometry_msgs
   multi_localizer_msgs
   nav_msgs
   object_detector_msgs
   roscpp
   rospy
   std_msgs
   tf2_geometry_msgs
   tf2_ros
   visualization_msgs
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

# object_localizer_node
add_executable(object_localizer_node
   ros_utils/dynamic_objects.cpp
   ros_utils/color.cpp
   ros_utils/object_param.cpp
   ros_utils/object.cpp
   ros_utils/objects.cpp
   ros_utils/object_map.cpp
   src/particle.cpp
   src/mcl.cpp
   src/mcl_node.cpp
)
target_link_libraries(object_localizer_node ${catkin_LIBRARIES})