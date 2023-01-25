#ifndef AMCL_POSE_REPUBLISHER_H_
#define AMCL_POSE_REPUBLISHER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

// Custom msg
#include "object_detector_msgs/ObjectPositions.h"
#include "multi_localizer_msgs/ObjectsData.h"
#include "multi_localizer_msgs/RobotPoseStamped.h"

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
    void od_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);

    void filter_od(object_detector_msgs::ObjectPositions input_od,object_detector_msgs::ObjectPositions& output_od);
    bool is_visible_range(object_detector_msgs::ObjectPosition od);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber od_sub_;

    // publisher
    ros::Publisher pose_pub_;
    ros::Publisher data_pub_;

    // tf
    boost::shared_ptr<tf2_ros::Buffer> buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> listener_;
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // buffer
    ros::Time start_time_;
    multi_localizer_msgs::RobotPoseStamped pose_;
    nav_msgs::Odometry odom_;

    // params
    std::string ROBOT_NAME_;
    std::string MAP_FRAME_ID_;
    std::string BASE_LINK_FRAME_ID_;
    bool PUBLISH_OBJECTS_DATA_;
    double PROBABILITY_TH_;
    double ANGLE_OF_VIEW_;
    double VISIBLE_LOWER_DISTANCE_;
    double VISIBLE_UPPER_DISTANCE_;
};
} // namespace multi_localizer

#endif  // AMCL_POSE_REPUBLISHER_H_
