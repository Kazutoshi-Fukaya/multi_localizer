cmake_minimum_required(VERSION 3.0.2)
project(multi_localizer)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
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

# pose_recorder
add_executable(pose_recorder src/pose_recorder.cpp)
target_link_libraries(pose_recorder ${catkin_LIBRARIES})