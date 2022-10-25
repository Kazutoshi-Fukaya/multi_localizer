#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

// Custom msg
#include "multi_robot_msgs/ObjectsData.h"

namespace multi_localizer
{
class AMCLPoseRepublisher
{
public:
	AMCLPoseRepublisher();
	void process();

private:
	void odom_callback(const nav_msgs::OdometryConstPtr& msg);
	void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

	// node handler
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// subscriber
	ros::Subscriber pose_sub_;
	ros::Subscriber odom_sub_;

	// publisher
	ros::Publisher pose_pub_;

	// tf
	boost::shared_ptr<tf2_ros::Buffer> buffer_;
	boost::shared_ptr<tf2_ros::TransformListener> listener_;
	boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

	// buffer
	nav_msgs::Odometry odom_;

	// parameter
	bool PUBLISH_OBJ_MSG_;
	std::string MAP_FRAME_ID_;
	std::string BASE_LINK_FRAME_ID_;
};
}

multi_localizer::AMCLPoseRepublisher::AMCLPoseRepublisher() :
	private_nh_("~")
{
	private_nh_.param("MAP_FRAME_ID",MAP_FRAME_ID_,{std::string("map")});
	private_nh_.param("BASE_LINK_FRAME_ID",BASE_LINK_FRAME_ID_,{std::string("base_link")});

	pose_sub_ = nh_.subscribe("pose_in",1,&AMCLPoseRepublisher::pose_callback,this);
	odom_sub_ = nh_.subscribe("odom_in",1,&AMCLPoseRepublisher::odom_callback,this);

	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_out",1);

	private_nh_.param("PUBLISH_OBJ_MSG",PUBLISH_OBJ_MSG_,{false});
	if(PUBLISH_OBJ_MSG_){
		// TO DO 
	}

	buffer_.reset(new tf2_ros::Buffer);
	listener_.reset(new tf2_ros::TransformListener(*buffer_));
	broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

void multi_localizer::AMCLPoseRepublisher::odom_callback(const nav_msgs::OdometryConstPtr& msg) { odom_ = *msg; }

void multi_localizer::AMCLPoseRepublisher::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	geometry_msgs::PoseStamped pose;
	pose.header = msg->header;
	pose.pose = msg->pose.pose;
	pose_pub_.publish(pose);

	tf2::Quaternion q;
    q.setRPY(0.0,0.0,tf2::getYaw(pose.pose.orientation));
    tf2::Transform map_to_robot(q,tf2::Vector3(pose.pose.position.x,pose.pose.position.y,0.0));
    
    geometry_msgs::PoseStamped robot_to_map_pose;
    robot_to_map_pose.header.frame_id = BASE_LINK_FRAME_ID_;
    robot_to_map_pose.header.stamp = odom_.header.stamp;
    tf2::toMsg(map_to_robot.inverse(),robot_to_map_pose.pose);

	geometry_msgs::PoseStamped odom_to_map_pose;
    try{
        buffer_->transform(robot_to_map_pose,odom_to_map_pose,odom_.header.frame_id);
    }
    catch(tf2::TransformException& ex){
        ROS_WARN("%s", ex.what());
        return;
    }
    tf2::Transform odom_to_map;
    tf2::convert(odom_to_map_pose.pose,odom_to_map);
    geometry_msgs::TransformStamped map_to_odom_transform;
    map_to_odom_transform.header.stamp = odom_.header.stamp;
    map_to_odom_transform.header.frame_id = MAP_FRAME_ID_;
    map_to_odom_transform.child_frame_id = odom_.header.frame_id;
    tf2::convert(odom_to_map.inverse(),map_to_odom_transform.transform);
    broadcaster_->sendTransform(map_to_odom_transform);
}

void multi_localizer::AMCLPoseRepublisher::process() { ros::spin(); }

int main(int argc,char** argv)
{
	ros::init(argc,argv,"amcl_pose");
	multi_localizer::AMCLPoseRepublisher amcl_pose_republisher;
	amcl_pose_republisher.process();
	return 0;
}