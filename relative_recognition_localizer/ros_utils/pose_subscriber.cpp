#include "ros_utils/pose_subscribers/pose_subscriber.h"

using namespace relative_recognition_localizer;

PoseSubscriber::PoseSubscriber()
{
	ROS_WARN("'pose_subscriber' has not received 'nh' and 'topic_name'");
}

PoseSubscriber::PoseSubscriber(ros::NodeHandle nh,std::string topic_name)
{
	pose_sub_ = nh.subscribe(topic_name,10,&PoseSubscriber::pose_callback,this);
}

void PoseSubscriber::pose_callback(const multi_localizer_msgs::RobotPoseStampedConstPtr msg)
{
    pose_ = *msg;
    has_received_pose_ = true;
}

void PoseSubscriber::set_received_flag(bool flag) 
{ 
	has_received_pose_ = flag; 
}

bool PoseSubscriber::get_pose(multi_localizer_msgs::RobotPoseStamped& pose)
{
	if(has_received_pose_){
		pose = pose_;
        return true;
    }
    return false;
}