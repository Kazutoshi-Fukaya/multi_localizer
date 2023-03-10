#ifndef MULTI_LOCALIZER_H_
#define MULTI_LOCALIZER_H_

#include <visualization_msgs/MarkerArray.h>

// mcl base
#include "mcl_base/mcl_base.h"

// ros utils
#include "ros_utils/robot_name_list/robot_name_list.h"
#include "ros_utils/pose_subscribers/pose_subscribers.h"

// utils
#include "utils/object_map/object_map.h"

// c++
#include <random>
#include <fstream>
#include <sstream>

// Custom msg
#include "multi_localizer_msgs/ObjectMap.h"
#include "multi_localizer_msgs/ObjectsData.h"
#include "multi_localizer_msgs/RobotPoseStamped.h"
#include "object_detector_msgs/ObjectPositions.h"
#include "object_color_detector_msgs/ObjectColorPositions.h"
#include "place_recognition_msgs/PoseStamped.h"

namespace multi_localizer
{
class MultiLocalizer : public MCLBase
{
public:
    MultiLocalizer();
    ~MultiLocalizer();
    void process();

private:
    void pr_callback(const place_recognition_msgs::PoseStampedConstPtr& msg);
    void od_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);
    void ocd_callback(const object_color_detector_msgs::ObjectColorPositionsConstPtr& msg);
    void object_map_callback(const multi_localizer_msgs::ObjectMapConstPtr& msg);

    // mcl base
    void observation_update();
    void publish_tf();
    bool is_start();
    bool is_observation();  // no use
    double get_weight(geometry_msgs::PoseStamped& pose);

    // place recognition
    bool is_pr_observation();

    // object detection
    void load_object_params(ObjectMap* object_map);
    void load_init_objects(ObjectMap* object_map);
    void filter_od_msg(object_detector_msgs::ObjectPositions input_od,
                       object_detector_msgs::ObjectPositions& output_od);
    void publish_objects_data();
    void publish_markers();
    bool is_od_observation();
    bool is_visible_range(object_detector_msgs::ObjectPosition op);

    // mutual recognition
    bool is_mr_observation();

    // utils
    std::vector<std::string> split(std::string& input,char delimiter);
    double weight_func(double mu,double sigma);

    // subscriber
    ros::Subscriber pr_pose_sub_;
    ros::Subscriber od_sub_;
    ros::Subscriber ocd_sub_;
    ros::Subscriber object_map_sub_;

    // publisher (for publishing obj data)
    ros::Publisher markers_pub_;
    ros::Publisher data_pub_;

    // time
    ros::Time start_time_;

    // ros utils
    RobotNameList* robot_name_list_;
    ObjectMap* object_map_;
    PoseSubscribers* pose_subscribers_;

    // buffer
    object_detector_msgs::ObjectPositions od_;
    object_color_detector_msgs::ObjectColorPositions ocd_;
    place_recognition_msgs::PoseStamped pr_pose_;
    bool has_got_weight_;

    // place recognition params
    bool USE_PLACE_RECOGNITION_;
    double ERROR_TH_;
    double SCORE_TH_;

    // object detection params
    bool USE_OBJECT_DETECTION_;
    bool PUBLISH_OBJECTS_DATA_;
    bool PUBLISH_MARKERS_;
    bool SUBSCRIBE_OBJECT_DATA_;
    std::string ROBOT_NAME_;
    double PROBABILITY_TH_;
    double VISIBLE_LOWER_DISTANCE_;
    double VISIBLE_UPPER_DISTANCE_;
    double ANGLE_OF_VIEW_;
    double DISTANCE_NOISE_;
    double SIM_TH_;

    // mutual recognition params
    bool USE_MUTUAL_RECOGNITION_;

    // Weight
    double ALPHA_;
    double BETA_;
};
} // namespace multi_localizer

#endif  // MULTI_LOCALIZER_H_
