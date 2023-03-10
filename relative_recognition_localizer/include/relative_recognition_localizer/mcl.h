#ifndef MCL_H_
#define MCL_H_

// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

// c++
#include <random>

// utils
#include "ros_utils/pose_subscribers/pose_subscribers.h"

// Custom msg
#include "object_color_detector_msgs/ObjectColorPositions.h"

namespace relative_recognition_localizer
{
class MCL
{
public:
    MCL();
    ~MCL();
    void process();

private:
    class Particle
    {
    public:
        Particle(MCL* mcl);

        void set_pose(double x,double y,double yaw,double x_var,double y_var,double yaw_var);
        void move(double dx,double dy,double dyaw);

        geometry_msgs::PoseStamped pose_;
        double weight_;
    private:
        MCL* mcl_;
    };

private:
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    void ocd_callback(const object_color_detector_msgs::ObjectColorPositionsConstPtr& msg);

    void init();
    void init_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw);
    void set_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw);
    void set_particles(double x,double y,double yaw,double x_var,double y_var,double yaw_var);
    void motion_update();
    void observation_update();
    void normalize_particles_weight();
    void calc_weight_params();
    void resample_particles();
    void sort_by_particle_weight();
    void calc_estimated_pose();
    void calc_variance();
    void publish_particle_poses();
    void publish_estimated_pose();
    void publish_tf();

    Particle generate_particle();
    geometry_msgs::Quaternion get_quat_msg_from_yaw(double yaw);
    bool is_start();
    bool is_dense();
    bool is_spread();
    bool is_observation();
    double get_gaussian(double mu,double sigma);
    double get_angle_diff(double a,double b);
    double get_weight(geometry_msgs::PoseStamped& pose);
    double weight_func(double mu,double sigma);
    double get_particle_weight_sum();
    double get_max_particle_weight();

    // node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    PoseSubscribers* pose_subs_;
    ros::Subscriber odom_sub_;
    ros::Subscriber ocd_sub_;

    // publisher
    ros::Publisher pose_pub_;
    ros::Publisher poses_pub_;

    // random
    std::random_device seed_;
    std::mt19937 engine_;

    // tf
    boost::shared_ptr<tf2_ros::Buffer> buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> listener_;
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // particles
    std::vector<Particle> particles_;

    // buffer
    ros::Time start_time_;
    object_color_detector_msgs::ObjectColorPositions ocd_;
    geometry_msgs::PoseStamped estimated_pose_;
    geometry_msgs::PoseStamped current_odom_;
    geometry_msgs::PoseStamped previous_odom_;
    bool has_received_odom_;
    bool is_update_;
    double x_var_;
    double y_var_;
    double yaw_var_;
    double weight_average_;
    double weight_slow_;
    double weight_fast_;
    double distance_sum_;
    double angle_sum_;

    // params
    std::string MAP_FRAME_ID_;
    std::string BASE_LINK_FRAME_ID_;
    bool IS_TF_;
    int HZ_;
    int PARTICLES_NUM_;
    double INIT_X_;
    double INIT_Y_;
    double INIT_YAW_;
    double INIT_X_VAR_;
    double INIT_Y_VAR_;
    double INIT_YAW_VAR_;
    double LOWER_X_VAR_TH_;
    double LOWER_Y_VAR_TH_;
    double LOWER_YAW_VAR_TH_;
    double UPPER_X_VAR_TH_;
    double UPPER_Y_VAR_TH_;
    double UPPER_YAW_VAR_TH_;
    double ALPHA_1_;
    double ALPHA_2_;
    double ALPHA_3_;
    double ALPHA_4_;
    double ALPHA_SLOW_;
    double ALPHA_FAST_;
    double DISTANCE_TH_;
    double ANGLE_TH_;
    double SELECTION_RATIO_;
    double DISTANCE_NOISE_;
};
} // namespace relative_recognition_localizer

#endif  // MCL_H_