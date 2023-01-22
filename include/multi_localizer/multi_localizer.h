#ifndef MULTI_LOCALIZER_H_
#define MULTI_LOCALIZER_H_

// mcl base
#include "mcl_base/mcl_base.h"

// #include "ros_utils/recorder/recorder.h"
#include "ros_utils/robot_name_list/robot_name_list.h"
#include "utils/object_map/object_map.h"
#include "pose_subscribers/pose_subscribers.h"

// c++
#include <random>
#include <fstream>
#include <sstream>


// Custom msg
#include "multi_robot_msgs/ObjectMap.h"
#include "multi_robot_msgs/ObjectsData.h"
#include "object_detector_msgs/ObjectPositions.h"
#include "object_color_detector_msgs/ObjectColorPositions.h"

namespace multi_localizer
{
class MultiLocalizer : public MCLBase
{
public:
    MultiLocalizer();
    ~MultiLocalizer();
    void process();

private:
    void od_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);
    void ocd_callback(const object_color_detector_msgs::ObjectColorPositionsConstPtr& msg);

    // mcl base
    void observation_update();
    void publish_tf();
    bool is_start();
    bool is_observation();
    double get_weight(geometry_msgs::PoseStamped& pose);

    // object detection
    void filter_ops_msg(object_detector_msgs::ObjectPositions input_ops,
                        object_detector_msgs::ObjectPositions& output_ops);
    void publish_objects_msg();
    void observation_od_update();

    bool is_visible_range(object_detector_msgs::ObjectPosition op);

    // utils
    double weight_func(double mu,double sigma);

    // subscriber
    ros::Subscriber ops_sub_;
    ros::Subscriber ocps_sub_;


    // publisher (for publishing obj data)
    ros::Publisher obj_pub_;

    // time
    ros::Time start_time_;

    // utility
    RobotNameList* robot_name_list_;
    ObjectMap* object_map_;
    PoseSubscribers* pose_subscribers_;

    // buffer
    object_detector_msgs::ObjectPositions ops_;
    object_color_detector_msgs::ObjectColorPositions ocps_;
    geometry_msgs::PoseStamped ref_pose_;	// for recording

    // object detection params
    bool USE_OPS_MSG_;
    bool USE_OCPS_MSG_;
    bool PUBLISH_DATABASE_;
    bool PUBLISH_OBJ_DATA_;
    double PROBABILITY_TH_;
    double VISIBLE_LOWER_DISTANCE_;
    double VISIBLE_UPPER_DISTANCE_;
    double ANGLE_OF_VIEW_;
    double DISTANCE_NOISE_;
};
} // namespace multi_localizer

#endif  // MULTI_LOCALIZER_H_
