cmake_minimum_required(VERSION 3.0.2)
project(multi_localizer)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  darknet_ros_msgs
  geometry_msgs
  multi_localizer_msgs
  nav_msgs
  object_color_detector_msgs
  object_detector_msgs
  place_recognition_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

# OpenCV
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.0 QUIET)
  if(NOT OpenCV_FOUND)
    find_package(OpenCV 3.4.3 QUIET)
    if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# mcl_base
SET(mcl_base_hdrs
  include/mcl_base/mcl_base.h
)
SET(mcl_base_srcs
  mcl_base/mcl_base.cpp
  mcl_base/particle.cpp
)
ADD_LIBRARY(mcl_base
  ${mcl_base_srcs}
  ${mcl_base_hdrs}
)

# object_map
SET(object_map_hdrs
  include/ros_utils/object_map/color.h
  include/ros_utils/object_map/object_param.h
  include/ros_utils/object_map/object.h
  include/ros_utils/object_map/objects.h
  include/ros_utils/object_map/object_map.h
)
SET(object_map_srcs
  ros_utils/object_map/color.cpp
  ros_utils/object_map/object_param.cpp
  ros_utils/object_map/object.cpp
  ros_utils/object_map/objects.cpp
  ros_utils/object_map/object_map.cpp
)
ADD_LIBRARY(object_map
  ${object_map_srcs}
  ${object_map_hdrs}
)

# robot_name_list
SET(robot_name_list_hdrs
  include/ros_utils/robot_name_list/robot_name_list.h
)
SET(robot_name_list_srcs
  ros_utils/robot_name_list/robot_name_list.cpp
)
ADD_LIBRARY(robot_name_list
  ${robot_name_list_srcs}
  ${robot_name_list_hdrs}
)

# pose_subscribers
SET(pose_subscribers_hdrs
  include/ros_utils/pose_subscribers/robot_element.h
  include/ros_utils/pose_subscribers/pose_subscriber.h
  include/ros_utils/pose_subscribers/pose_subscribers.h
)
SET(pose_subscribers_srcs
  ros_utils/pose_subscribers/robot_element.cpp
  ros_utils/pose_subscribers/pose_subscriber.cpp
  ros_utils/pose_subscribers/pose_subscribers.cpp
)
ADD_LIBRARY(pose_subscribers
  ${pose_subscribers_srcs}
  ${pose_subscribers_hdrs}
)

# multi_localizer
add_executable(multi_localizer_node
  src/multi_localizer.cpp
  src/multi_localizer_node.cpp
)
target_link_libraries(multi_localizer_node
  ${catkin_LIBRARIES}
  mcl_base
  object_map
  robot_name_list
  pose_subscribers
)
