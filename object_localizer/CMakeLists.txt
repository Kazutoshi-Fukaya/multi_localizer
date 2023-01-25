cmake_minimum_required(VERSION 3.0.2)
project(object_localizer)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    multi_robot_msgs
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

# tf_broadcaster
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

# object_localizer_node
add_executable(object_localizer_node src/mcl.cpp)
target_link_libraries(object_localizer_node ${catkin_LIBRARIES})
