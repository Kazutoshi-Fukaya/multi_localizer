#ifndef POSE_SUBSCRIBERS_H_
#define POSE_SUBSCRIBERS_H_

#include <tf2/utils.h>
#include <map>

#include "ros_utils/pose_subscribers/robot_element.h"
#include "ros_utils/pose_subscribers/pose_subscriber.h"

namespace multi_localizer
{
class PoseSubscribers : public std::map<RobotElement*,PoseSubscriber*>
{
public:
    PoseSubscribers();
    PoseSubscribers(ros::NodeHandle nh,ros::NodeHandle private_nh);

    void set_received_flag(std::string color,bool flag);
    void debug(std::string color);

    std::string get_robot_name(std::string color);
    std::string get_color(std::string robot_name);
    bool get_pose(std::string color,multi_localizer_msgs::RobotPoseStamped& pose);

private:
    void load_robot_element(ros::NodeHandle nh,ros::NodeHandle private_nh);
};
} // namespace multi_localizer

#endif  // POSE_SUBSCRIBERS_H_