#ifndef POSE_SUBSCRIBER_H_
#define POSE_SUBSCRIBER_H_

#include <ros/ros.h>

#include "multi_localizer_msgs/RobotPoseStamped.h"

namespace relative_recognition_localizer
{
class PoseSubscriber
{
public:
    PoseSubscriber() :
        topic_name_(std::string("")) { }

    PoseSubscriber(ros::NodeHandle _nh,std::string _topic_name) :
        nh_(_nh), topic_name_(_topic_name) { setup_subscriber(); }

    bool get_pose(multi_localizer_msgs::RobotPoseStamped& pose)
    {
        if(has_received_pose_){
            pose = pose_;
            return true;
        }
        return false;
    }

    void set_received_flag(bool flag) { has_received_pose_ = flag; }

private:
    void pose_callback(const multi_localizer_msgs::RobotPoseStampedConstPtr msg)
    {
        pose_ = *msg;
        has_received_pose_ = true;
    }

    void setup_subscriber() { pose_sub_ = nh_.subscribe(topic_name_,10,&PoseSubscriber::pose_callback,this); }

    // node handler
    ros::NodeHandle nh_;

    // subscriber
    ros::Subscriber pose_sub_;
    
    // buffer
    multi_localizer_msgs::RobotPoseStamped pose_;
    bool has_received_pose_;
    std::string topic_name_;
};
} // namespace relative_recognition_localizer

#endif  // POSE_SUBSCRIBER_H_