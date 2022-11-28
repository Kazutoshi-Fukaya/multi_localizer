cmake_minimum_required(VERSION 3.0.2)
project(multi_localizer)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    multi_robot_msgs
    nav_msgs
    object_color_detector_msgs
    object_detector_msgs
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

# tf_broadcaster
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

# single_mcl
add_executable(single_mcl 
    src/mcl_base.cpp
    src/particle.cpp
    src/single_mcl.cpp
)
target_link_libraries(single_mcl ${catkin_LIBRARIES})

# amcl_pose_republisher
add_executable(amcl_pose_republisher src/amcl_pose_republisher.cpp)
target_link_libraries(amcl_pose_republisher ${catkin_LIBRARIES})

# object_localizer
add_executable(object_localizer
    src/mcl_base.cpp
    src/particle.cpp
    src/object_localizer.cpp
)
target_link_libraries(object_localizer ${catkin_LIBRARIES})

# image_connector
add_executable(image_connector src/image_connector.cpp)
target_link_libraries(image_connector 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# semi_dynamic_map_creator
add_executable(semi_dynamic_map_creator src/semi_dynamic_map_creator.cpp)
target_link_libraries(semi_dynamic_map_creator ${catkin_LIBRARIES})