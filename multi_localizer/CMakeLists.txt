cmake_minimum_required(VERSION 3.0.2)
project(multi_localizer)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
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

add_library(ObjectMap SHARED
    utils/object.cpp
    utils/objects.cpp
    utils/object_map.cpp
)

# single_mcl
add_executable(single_mcl 
    src/mcl_base.cpp
    src/particle.cpp
    src/single_mcl.cpp
)
target_link_libraries(single_mcl ${catkin_LIBRARIES})

# pr_localizer
add_executable(pr_localizer
    src/mcl_base.cpp
    src/particle.cpp
    src/pr_localizer.cpp
)
target_link_libraries(pr_localizer ${catkin_LIBRARIES})

# object_localizer
add_executable(object_localizer
    src/mcl_base.cpp
    src/particle.cpp
    src/object_localizer.cpp
)
target_link_libraries(object_localizer ${catkin_LIBRARIES})

# multi_localizer
add_executable(multi_localizer
    src/mcl_base.cpp
    src/particle.cpp
    src/multi_localizer.cpp
)
target_link_libraries(multi_localizer 
    ${catkin_LIBRARIES}
    ObjectMap
)

# ============================== UTILS ==============================

# images_connector
add_executable(images_connector 
    ros_utils/image_subscriber.cpp
    ros_utils/images_connector.cpp)
target_link_libraries(images_connector 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# tf_broadcaster
add_executable(tf_broadcaster ros_utils/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

# amcl_pose_republisher
add_executable(amcl_pose_republisher ros_utils/amcl_pose_republisher.cpp)
target_link_libraries(amcl_pose_republisher ${catkin_LIBRARIES})

# recorder
add_executable(recorder ros_utils/recorder.cpp)
target_link_libraries(recorder ${catkin_LIBRARIES})

# semi_dynamic_map_creator
# add_executable(semi_dynamic_map_creator src/semi_dynamic_map_creator.cpp)
# target_link_libraries(semi_dynamic_map_creator ${catkin_LIBRARIES})