#ifndef MULTI_LOCALIZER_H_
#define MULTI_LOCALIZER_H_

// mcl base
#include "mcl_base/mcl_base.h"

#include "ros_utils/recorder/recorder.h"
#include "ros_utils/robot_name_list/robot_name_list.h"
#include "utils/object_map/object_map.h"
#include "pose_subscribers/pose_subscribers.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
    void ops_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);
    void ocps_callback(const object_color_detector_msgs::ObjectColorPositionsConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void observation_update();
    void publish_tf();
    bool is_start();
    bool is_observation();
    double get_weight(geometry_msgs::PoseStamped& pose);

    void filter_ops_msg(object_detector_msgs::ObjectPositions input_ops,
                        object_detector_msgs::ObjectPositions& output_ops);
    void publish_objects_msg();

    // subscriber
    ros::Subscriber ops_sub_;
    ros::Subscriber ocps_sub_;

    // subscriber (for recording)
    ros::Subscriber pose_sub_;

    // publisher (for publishing obj data)
    ros::Publisher obj_pub_;

    // time
    ros::Time start_time_;

    // utility
    RobotNameList* robot_name_list_;
    ObjectMap* object_map_;
    PoseSubscribers* pose_subscribers_;
    Recorder* recorder_;

    // buffer
    object_detector_msgs::ObjectPositions ops_;
    object_color_detector_msgs::ObjectColorPositions ocps_;
    geometry_msgs::PoseStamped ref_pose_;	// for recording

    // parameters
    bool USE_OPS_MSG_;
    bool USE_OCPS_MSG_;
    bool PUBLISH_DATABASE_;
    bool PUBLISH_OBJ_DATA_;
    bool IS_RECORD_;
    double PROBABILITY_TH_;
    double VISIBLE_LOWER_DISTANCE_;
    double VISIBLE_UPPER_DISTANCE_;
    double ANGLE_OF_VIEW_;
    double DISTANCE_NOISE_;
};
} // namespace multi_localizer

#endif  // MULTI_LOCALIZER_H_
