#ifndef POSE_SUBSCRIBER_H_
#define POSE_SUBSCRIBER_H_

#include <ros/ros.h>

#include "multi_localizer_msgs/RobotPoseStamped.h"

namespace relative_recognition_localizer
{
class PoseSubscriber
{
public:
    PoseSubscriber();
    PoseSubscriber(ros::NodeHandle nh,std::string topic_name);

    void set_received_flag(bool flag);
    bool get_pose(multi_localizer_msgs::RobotPoseStamped& pose);

private:
    void pose_callback(const multi_localizer_msgs::RobotPoseStampedConstPtr msg);
    
    // subscriber
    ros::Subscriber pose_sub_;
    
    // buffer
    multi_localizer_msgs::RobotPoseStamped pose_;
    bool has_received_pose_;
};
} // namespace relative_recognition_localizer

#endif  // POSE_SUBSCRIBER_H_