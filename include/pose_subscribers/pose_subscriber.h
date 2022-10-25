#ifndef POSE_SUBSCRIBER_H_
#define POSE_SUBSCRIBER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class PoseSubscriber
{
public:
    PoseSubscriber() :
        topic_name_(std::string("")) { setup_subscriber(); }

    PoseSubscriber(ros::NodeHandle _nh,std::string _topic_name) :
        nh_(_nh), topic_name_(_topic_name) { setup_subscriber(); }

    bool get_pose(geometry_msgs::PoseStamped& pose)
    {
        if(has_received_pose_){
            pose = pose_;
            return true;
        }
        return false;
    }

    void set_received_flag(bool flag) { has_received_pose_ = flag; }

private:
    void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_ = *msg;
        has_received_pose_ = true;
    }

    void setup_subscriber()
    {
        pose_sub_ = nh_.subscribe(topic_name_,10,&PoseSubscriber::pose_callback,this);
    }

    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    geometry_msgs::PoseStamped pose_;
    bool has_received_pose_;
    std::string topic_name_;
};

#endif  // POSE_SUBSCRIBER_H_
