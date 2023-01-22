#ifndef TF_BROADCASTER_H_
#define TF_BROADCASTER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

// Custom msg
#include "object_detector_msgs/ObjectPositions.h"
#include "object_color_detector_msgs/ObjectColorPositions.h"

namespace multi_localizer
{
class TFBroadcaster
{
public:
    TFBroadcaster();
    void process();

private:
    struct Pose
    {
        Pose() :
            x(0.0), y(0.0), z(0.0),
            roll(0.0), pitch(0.0), yaw(0.0) {}

        Pose(double _x,double _y,double _z,double _roll,double _pitch,double _yaw) :
            x(_x), y(_y), z(_z),
            roll(_roll), pitch(_pitch), yaw(_yaw) {}

        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };

private:
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    geometry_msgs::TransformStamped get_transfrom_stamped(std::string frame_id,std::string child_frame_id,Pose pose);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber odom_sub_;

    // tf
    boost::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // buffer
    geometry_msgs::TransformStamped laser_transform_stamped_;
    geometry_msgs::TransformStamped camera_transform_stamped_;

    // params
    std::string BASE_LINK_FRAME_ID_;
    std::string LASER_FRAME_ID_;
    std::string CAMERA_FRAME_ID_;
};	
} // namespace multi_localizer

#endif	// TF_BROADCASTER_H_