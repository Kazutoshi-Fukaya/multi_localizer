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

# MCL Base
add_library(MCLBase SHARED
    mcl_base/mcl_base.cpp
    mcl_base/particle.cpp
)

# Object Map
add_library(ObjectMap SHARED
    utils/object.cpp
    utils/objects.cpp
    utils/object_map.cpp
)

# multi_localizer
add_executable(multi_localizer src/multi_localizer.cpp)
target_link_libraries(multi_localizer 
    ${catkin_LIBRARIES}
    MCLBase
    ObjectMap
)