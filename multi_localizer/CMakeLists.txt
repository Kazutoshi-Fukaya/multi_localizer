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

# ============================== MAIN ==============================

# mcl
add_executable(mcl src/mcl.cpp)
target_link_libraries(mcl 
    ${catkin_LIBRARIES}
    MCLBase
)

# pr_localizer
add_executable(pr_localizer src/pr_localizer.cpp)
target_link_libraries(pr_localizer 
    ${catkin_LIBRARIES}
    MCLBase
)

# multi_localizer
add_executable(multi_localizer src/multi_localizer.cpp)
target_link_libraries(multi_localizer 
    ${catkin_LIBRARIES}
    MCLBase
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

# image_saver
add_executable(image_saver ros_utils/image_saver.cpp)
target_link_libraries(image_saver 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# bbox_msg_recorder
add_executable(bbox_msg_recorder ros_utils/bbox_msg_recorder.cpp)
target_link_libraries(bbox_msg_recorder ${catkin_LIBRARIES})

# pose_msg_recorder
add_executable(pose_msg_recorder ros_utils/pose_msg_recorder.cpp)
target_link_libraries(pose_msg_recorder ${catkin_LIBRARIES})

# sampler
add_executable(sampler ros_utils/sampler.cpp)
target_link_libraries(sampler ${catkin_LIBRARIES})

# semi_dynamic_map_creator
# add_executable(semi_dynamic_map_creator src/semi_dynamic_map_creator.cpp)
# target_link_libraries(semi_dynamic_map_creator ${catkin_LIBRARIES})