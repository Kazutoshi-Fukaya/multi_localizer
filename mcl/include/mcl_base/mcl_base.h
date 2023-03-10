#ifndef MCL_BASE_H_
#define MCL_BASE_H_

// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

// c++
#include <random>

namespace mcl
{
class MCLBase
{
public:
    MCLBase();
    virtual void process() = 0;

protected:
    class Particle
    {
    public:
        Particle(MCLBase* mcl);

        void set_pose(double x,double y,double yaw,double x_var,double y_var,double yaw_var);
        void move(double dx,double dy,double dyaw);

        geometry_msgs::PoseStamped pose_;
        double weight_;
    private:
        MCLBase* mcl_;
    };

    void odom_callback(const nav_msgs::OdometryConstPtr& msg);

    virtual void observation_update() = 0;
    virtual void publish_tf() = 0;
    virtual bool is_start() = 0;
    virtual double get_weight(geometry_msgs::PoseStamped& pose) = 0;


    void init();
    void init_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw);
    void set_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw);
    void set_particles(double x,double y,double yaw,double x_var,double y_var,double yaw_var);
    void motion_update();
    void normalize_particles_weight();
    void calc_weight_params();
    void resample_particles();
    void sort_by_particle_weight();
    void calc_estimated_pose();
    void calc_variance();
    void publish_particle_poses();
    void publish_estimated_pose();

    Particle generate_particle();
    geometry_msgs::Quaternion get_quat_msg_from_yaw(double yaw);
    bool is_dense();
    bool is_spread();
    double get_gaussian(double mu,double sigma);
    double get_angle_diff(double a,double b);
    double get_particle_weight_sum();
    double get_max_particle_weight();

    // node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber odom_sub_;

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

    // parameters
    std::string MAP_FRAME_ID_;
    std::string BASE_LINK_FRAME_ID_;
    bool IS_TF_;
    int HZ_;
    int PARTICLES_NUM;
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
};
} // namespace mcl

#endif  // MCL_BASE_H_