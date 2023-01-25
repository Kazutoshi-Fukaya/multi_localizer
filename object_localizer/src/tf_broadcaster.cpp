#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

// Custom msg
#include "object_detector_msgs/ObjectPositions.h"

namespace object_localizer
{
class TfBroadcaster
{
public:
	TfBroadcaster();
    void process();

private:
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
	
    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber odom_sub_;

    // tf
    boost::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // parameters
    std::string BASE_LINK_FRAME_ID_;
};
}

object_localizer::TfBroadcaster::TfBroadcaster() :
	private_nh_("~")
{
    private_nh_.param("BASE_LINK_FRAME_ID",BASE_LINK_FRAME_ID_,{std::string("base_link")});

    odom_sub_ = nh_.subscribe("odom_in",1,&TfBroadcaster::odom_callback,this);

    static_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster);
    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

void object_localizer::TfBroadcaster::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    geometry_msgs::TransformStamped odom_transform;
    odom_transform.header = msg->header;
    odom_transform.child_frame_id = msg->child_frame_id;
    odom_transform.transform.translation.x = msg->pose.pose.position.x;
    odom_transform.transform.translation.y = msg->pose.pose.position.y;
    odom_transform.transform.translation.z = msg->pose.pose.position.z;
    odom_transform.transform.rotation = msg->pose.pose.orientation;

    broadcaster_->sendTransform(odom_transform);
}

void object_localizer::TfBroadcaster::process() { ros::spin(); }

int main(int argc,char** argv)
{
    ros::init(argc,argv,"tf_broadcaster");
    object_localizer::TfBroadcaster tf_broadcaster;
    tf_broadcaster.process();
    return 0;
}