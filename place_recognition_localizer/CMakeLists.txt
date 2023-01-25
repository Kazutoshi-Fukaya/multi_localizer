cmake_minimum_required(VERSION 3.0.2)
project(place_recognition_localizer)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  multi_localizer_msgs
  nav_msgs
  place_recognition_msgs
  roscpp
  rospy
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
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

# place_recognition_localizer_node
add_executable(${PROJECT_NAME}_node
    src/particle.cpp
    src/mcl.cpp
    src/mcl_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
