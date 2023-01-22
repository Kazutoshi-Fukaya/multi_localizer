#include "ros_utils/tf_broadcaster/tf_broadcaster.h"

using namespace multi_localizer;

TFBroadcaster::TFBroadcaster() : private_nh_("~")
{
    private_nh_.param("BASE_LINK_FRAME_ID",BASE_LINK_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("LASER_FRAME_ID",LASER_FRAME_ID_,{std::string("laser")});
    private_nh_.param("CAMERA_FRAME_ID",CAMERA_FRAME_ID_,{std::string("camera")});

    Pose LASER_POSE;
    private_nh_.param("LASER_X",LASER_POSE.x,{0.0});
    private_nh_.param("LASER_Y",LASER_POSE.y,{0.0});
    private_nh_.param("LASER_Z",LASER_POSE.z,{0.0});
    private_nh_.param("LASER_ROLL",LASER_POSE.roll,{0.0});
    private_nh_.param("LASER_PITCH",LASER_POSE.pitch,{0.0});
    private_nh_.param("LASER_YAW",LASER_POSE.yaw,{0.0});
    laser_transform_stamped_ = get_transfrom_stamped(BASE_LINK_FRAME_ID_,LASER_FRAME_ID_,LASER_POSE);

    Pose CAMERA_POSE;
    private_nh_.param("CAMERA_X",CAMERA_POSE.x,{0.0});
    private_nh_.param("CAMERA_Y",CAMERA_POSE.y,{0.0});
    private_nh_.param("CAMERA_Z",CAMERA_POSE.z,{0.0});
    private_nh_.param("CAMERA_ROLL",CAMERA_POSE.roll,{0.0});
    private_nh_.param("CAMERA_PITCH",CAMERA_POSE.pitch,{0.0});
    private_nh_.param("CAMERA_YAW",CAMERA_POSE.yaw,{0.0});
    camera_transform_stamped_ = get_transfrom_stamped(BASE_LINK_FRAME_ID_,CAMERA_FRAME_ID_,CAMERA_POSE);

    odom_sub_ = nh_.subscribe("odom_in",1,&TFBroadcaster::odom_callback,this);

    static_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster);
    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

void TFBroadcaster::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    geometry_msgs::TransformStamped odom_transform;
    odom_transform.header =  msg->header;
    odom_transform.child_frame_id = msg->child_frame_id;
    odom_transform.transform.translation.x = msg->pose.pose.position.x;
    odom_transform.transform.translation.y = msg->pose.pose.position.y;
    odom_transform.transform.translation.z = msg->pose.pose.position.z;
    odom_transform.transform.rotation = msg->pose.pose.orientation;
    broadcaster_->sendTransform(odom_transform);
}

geometry_msgs::TransformStamped TFBroadcaster::get_transfrom_stamped(std::string frame_id,std::string child_frame_id,Pose pose)
{
    geometry_msgs::TransformStamped static_transform;
    static_transform.header.stamp = ros::Time::now();
    static_transform.header.frame_id = frame_id;
    static_transform.child_frame_id = child_frame_id;
    static_transform.transform.translation.x = pose.x;
    static_transform.transform.translation.y = pose.y;
    static_transform.transform.translation.z = pose.z;
    tf2::Quaternion tf_q;
    tf_q.setRPY(pose.roll,pose.pitch,pose.yaw);
    static_transform.transform.rotation.x = tf_q.x();
    static_transform.transform.rotation.y = tf_q.y();
    static_transform.transform.rotation.z = tf_q.z();
    static_transform.transform.rotation.w = tf_q.w();

    return static_transform;
}

void TFBroadcaster::process()
{
    static_broadcaster_->sendTransform({laser_transform_stamped_, camera_transform_stamped_});
    ros::spin();
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"tf_broadcaster");
    TFBroadcaster tf_broadcaster;
    tf_broadcaster.process();
    return 0;
}
