#include "relative_recognition_localizer/mcl.h"

using namespace relative_recognition_localizer;

MCL::MCL() :
    private_nh_("~"),
    pose_subs_(new PoseSubscribers(nh_,private_nh_)),
    engine_(seed_()),
    start_time_(ros::Time::now()),
    has_received_odom_(false), is_update_(false),
    x_var_(0.0), y_var_(0.0), yaw_var_(0.0),
    weight_average_(0.0), weight_slow_(0.0), weight_fast_(0.0),
    distance_sum_(0.0), angle_sum_(0.0)
{
    // params
    private_nh_.param("BASE_LINK_FRAME_ID",BASE_LINK_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("MAP_FRAME_ID",MAP_FRAME_ID_,{std::string("map")});
    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("PARTICLES_NUM",PARTICLES_NUM_,{2000});
    private_nh_.param("INIT_X",INIT_X_,{7.0});
    private_nh_.param("INIT_Y",INIT_Y_,{0.0});
    private_nh_.param("INIT_YAW",INIT_YAW_,{1.5708});
    private_nh_.param("INIT_X_VAR",INIT_X_VAR_,{0.5});
    private_nh_.param("INIT_Y_VAR",INIT_Y_VAR_,{0.5});
    private_nh_.param("INIT_YAW_VAR",INIT_YAW_VAR_,{0.5});
    private_nh_.param("UPPER_X_VAR_TH",UPPER_X_VAR_TH_,{0.80});
    private_nh_.param("UPPER_Y_VAR_TH",UPPER_Y_VAR_TH_,{0.80});
    private_nh_.param("UPPER_YAW_VAR_TH",UPPER_YAW_VAR_TH_,{0.70});
    private_nh_.param("LOWER_X_VAR_TH",LOWER_X_VAR_TH_,{0.15});
    private_nh_.param("LOWER_Y_VAR_TH",LOWER_Y_VAR_TH_,{0.15});
    private_nh_.param("LOWER_YAW_VAR_TH",LOWER_YAW_VAR_TH_,{0.10});
    private_nh_.param("ALPHA_1",ALPHA_1_,{0.3});
    private_nh_.param("ALPHA_2",ALPHA_2_,{0.3});
    private_nh_.param("ALPHA_3",ALPHA_3_,{0.1});
    private_nh_.param("ALPHA_4",ALPHA_4_,{0.1});
    private_nh_.param("ALPHA_SLOW",ALPHA_SLOW_,{0.001});
    private_nh_.param("ALPHA_FAST",ALPHA_FAST_,{0.1});
    private_nh_.param("DISTANCE_TH",DISTANCE_TH_,{0.15});
    private_nh_.param("ANGLE_TH",ANGLE_TH_,{0.15});
    private_nh_.param("SELECTION_RATIO",SELECTION_RATIO_,{0.2});
    private_nh_.param("DISTANCE_NOISE",DISTANCE_NOISE_,{0.1});

    ocd_sub_ = nh_.subscribe("ocd_in",1,&MCL::ocd_callback,this);
    odom_sub_ = nh_.subscribe("odom_in",1,&MCL::odom_callback,this);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_out",1);
    poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("poses_out",1);

    private_nh_.param("IS_TF",IS_TF_,{false});
    if(IS_TF_){
        buffer_.reset(new tf2_ros::Buffer);
        listener_.reset(new tf2_ros::TransformListener(*buffer_));
        broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    }

    init();
}

MCL::~MCL() {}

void MCL::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    if(IS_TF_){
        geometry_msgs::TransformStamped transform_stamped;
        try{
            transform_stamped = buffer_->lookupTransform(msg->header.frame_id,BASE_LINK_FRAME_ID_,ros::Time(0));
        }
        catch(tf2::TransformException& ex){
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        tf2::doTransform(pose,current_odom_,transform_stamped);
    }
    else{
        current_odom_.pose = msg->pose.pose;
    }
    current_odom_.header = msg->header;
    has_received_odom_ = true;
}

void MCL::ocd_callback(const object_color_detector_msgs::ObjectColorPositionsConstPtr& msg) { ocd_ = *msg; }

void MCL::init()
{
    set_particles(INIT_X_,INIT_Y_,INIT_YAW_,INIT_X_VAR_,INIT_Y_VAR_,INIT_YAW_VAR_);
    init_pose(previous_odom_,0.0,0.0,0.0);
    init_pose(current_odom_,0.0,0.0,0.0);
    init_pose(estimated_pose_,INIT_X_,INIT_Y_,INIT_YAW_);
}

void MCL::init_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw)
{
    pose.header.frame_id = MAP_FRAME_ID_;
    set_pose(pose,x,y,yaw);
}

void MCL::set_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw)
{
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = get_quat_msg_from_yaw(yaw);
}

void MCL::set_particles(double x,double y,double yaw,double x_var,double y_var,double yaw_var)
{
    std::vector<Particle> tmp_particles;
    for(int i = 0; i < PARTICLES_NUM_; i++){
        Particle particle = generate_particle();
        particle.set_pose(x,y,yaw,x_var,y_var,yaw_var);
        tmp_particles.emplace_back(particle);
    }
    particles_ = tmp_particles;
}

void MCL::motion_update()
{
    double dx = current_odom_.pose.position.x - previous_odom_.pose.position.x;
    double dy = current_odom_.pose.position.y - previous_odom_.pose.position.y;
    double dyaw = get_angle_diff(tf2::getYaw(current_odom_.pose.orientation),tf2::getYaw(previous_odom_.pose.orientation));

    distance_sum_ += std::sqrt(dx*dx + dy*dy);
    angle_sum_ += std::fabs(dyaw);

    if(distance_sum_ > DISTANCE_TH_ || angle_sum_ > ANGLE_TH_){
        distance_sum_ = 0.0;
        angle_sum_ = 0.0;
        is_update_ = true;
    }

    for(auto &p : particles_) p.move(dx,dy,dyaw);
}

void MCL::observation_update()
{
    if(!is_observation()) return;
    for(auto &p : particles_) p.weight_ = get_weight(p.pose_);
    normalize_particles_weight();
    calc_weight_params();
}

void MCL::normalize_particles_weight()
{
    double weight_sum = get_particle_weight_sum();
    weight_average_ = weight_sum/PARTICLES_NUM_;
    for(auto &p : particles_) p.weight_ /= weight_sum;
}

void MCL::calc_weight_params()
{
    if(weight_average_ ==  0 || std::isnan(weight_average_)){
        weight_average_ = 1 / (double)PARTICLES_NUM_;
        weight_slow_ = weight_average_;
        weight_fast_ = weight_average_;
    }

    if(weight_slow_ == 0.0){
        weight_slow_ = weight_average_;
    }
    else{
        weight_slow_ += ALPHA_SLOW_*(weight_average_ - weight_slow_);
    }

    if(weight_fast_ == 0.0){
        weight_fast_ = weight_average_;
    }
    else{
        weight_fast_ += ALPHA_FAST_*(weight_average_ - weight_fast_);
    }
}

void MCL::resample_particles()
{
    double weight;
    if((1 - weight_fast_/weight_slow_) > 0) weight = 1 - weight_fast_/weight_slow_;
    else weight = 0.0;

    std::uniform_real_distribution<> ur_dist(0.0,1.0);
    int index = (int)(ur_dist(engine_)*PARTICLES_NUM_);
    double beta = 0.0;
    double mv = get_max_particle_weight();
    std::vector<Particle> tmp_particles;
    for(int i = 0; i < (int)particles_.size(); i++){
        if(weight < ur_dist(engine_)){
            beta += 2.0*ur_dist(engine_)*mv;
            while(beta > particles_[index].weight_){
                beta -= particles_[index].weight_;
                index = (index + 1)%PARTICLES_NUM_;
            }
            tmp_particles.emplace_back(particles_[index]);
        }
        else{
            Particle particle = generate_particle();
            double tmp_x = estimated_pose_.pose.position.x;
            double tmp_y = estimated_pose_.pose.position.y;
            double tmp_yaw = tf2::getYaw(estimated_pose_.pose.orientation);
            particle.set_pose(tmp_x,tmp_y,tmp_yaw,INIT_X_VAR_,INIT_Y_VAR_,INIT_YAW_VAR_);
            tmp_particles.emplace_back(particle);
        }
    }
    particles_ = tmp_particles;
}

void MCL::sort_by_particle_weight()
{
    std::sort(particles_.begin(),particles_.end(),[](const Particle& x,const Particle& y){ return x.weight_ > y.weight_; });
}

void MCL::calc_estimated_pose()
{
    double tmp_est_x = 0.0;
    double tmp_est_y = 0.0;
    int num_of_choices = (int)(SELECTION_RATIO_*PARTICLES_NUM_);
    for(int i = 0; i < num_of_choices; i++){
        tmp_est_x += particles_[i].pose_.pose.position.x;
        tmp_est_y += particles_[i].pose_.pose.position.y;
    }
    tmp_est_x /= (double)num_of_choices;
    tmp_est_y /= (double)num_of_choices;
    double tmp_est_yaw = tf2::getYaw(particles_[0].pose_.pose.orientation);

    set_pose(estimated_pose_,tmp_est_x,tmp_est_y,tmp_est_yaw);
}

void MCL::calc_variance()
{
    double x_average = 0.0;
    double y_average = 0.0;
    double yaw_average = 0.0;
    for(const auto &p : particles_){
        x_average += p.pose_.pose.position.x;
        y_average += p.pose_.pose.position.y;
        yaw_average += tf2::getYaw(p.pose_.pose.orientation);
    }
    x_average /= (double)PARTICLES_NUM_;
    y_average /= (double)PARTICLES_NUM_;
    yaw_average /= (double)PARTICLES_NUM_;

    double x_rss = 0.0;
    double y_rss = 0.0;
    double yaw_rss = 0.0;
    for(const auto &p : particles_){
        x_rss += std::pow(p.pose_.pose.position.x - x_average,2);
        y_rss += std::pow(p.pose_.pose.position.y - y_average,2);
        yaw_rss += std::pow(tf2::getYaw(p.pose_.pose.orientation) - yaw_average,2);
    }
    x_var_ = std::sqrt(x_rss/(double)PARTICLES_NUM_);
    y_var_ = std::sqrt(y_rss/(double)PARTICLES_NUM_);
    yaw_var_ = std::sqrt(yaw_rss/(double)PARTICLES_NUM_);
}

void MCL::publish_particle_poses()
{
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = MAP_FRAME_ID_;
    poses.header.stamp = ros::Time::now();
    for(const auto &p : particles_) poses.poses.emplace_back(p.pose_.pose);
    poses_pub_.publish(poses);
}

void MCL::publish_estimated_pose()
{
    estimated_pose_.header.stamp = ros::Time::now();
    pose_pub_.publish(estimated_pose_);
}

void MCL::publish_tf()
{
    tf2::Quaternion q;
    q.setRPY(0.0,0.0,tf2::getYaw(estimated_pose_.pose.orientation));
    tf2::Transform map_to_robot(q,tf2::Vector3(estimated_pose_.pose.position.x,estimated_pose_.pose.position.y,0.0));

    geometry_msgs::PoseStamped robot_to_map_pose;
    robot_to_map_pose.header.frame_id = BASE_LINK_FRAME_ID_;
    robot_to_map_pose.header.stamp = current_odom_.header.stamp;
    tf2::toMsg(map_to_robot.inverse(),robot_to_map_pose.pose);

    geometry_msgs::PoseStamped odom_to_map_pose;
    try{
        buffer_->transform(robot_to_map_pose,odom_to_map_pose,current_odom_.header.frame_id);
    }
    catch(tf2::TransformException& ex){
        ROS_WARN("%s", ex.what());
        return;
    }
    tf2::Transform odom_to_map;
    tf2::convert(odom_to_map_pose.pose,odom_to_map);
    geometry_msgs::TransformStamped map_to_odom_transform;
    map_to_odom_transform.header.stamp = current_odom_.header.stamp;
    map_to_odom_transform.header.frame_id = MAP_FRAME_ID_;
    map_to_odom_transform.child_frame_id = current_odom_.header.frame_id;
    tf2::convert(odom_to_map.inverse(),map_to_odom_transform.transform);
    broadcaster_->sendTransform(map_to_odom_transform);
}

MCL::Particle MCL::generate_particle() { return MCL::Particle(this); }

geometry_msgs::Quaternion MCL::get_quat_msg_from_yaw(double yaw)
{
    geometry_msgs::Quaternion quat_msg;
    tf2::Quaternion tf_q;
    tf_q.setRPY(0.0,0.0,yaw);
    quat_msg.x = (double)tf_q.x();
    quat_msg.y = (double)tf_q.y();
    quat_msg.z = (double)tf_q.z();
    quat_msg.w = (double)tf_q.w();
    return quat_msg;
}

bool MCL::is_start() 
{
    if(has_received_odom_) return true;
    return false;
}

bool MCL::is_dense()
{
    if(x_var_ < LOWER_X_VAR_TH_ || y_var_ < LOWER_Y_VAR_TH_ || yaw_var_ < LOWER_YAW_VAR_TH_){
        x_var_ = INIT_X_VAR_;
        y_var_ = INIT_Y_VAR_;
        yaw_var_ = INIT_YAW_VAR_;
        return true;
    }
    return false;
}

bool MCL::is_spread()
{
    if(x_var_ > UPPER_X_VAR_TH_ || y_var_ > UPPER_Y_VAR_TH_ || yaw_var_ > UPPER_YAW_VAR_TH_){
        x_var_ = INIT_X_VAR_;
        y_var_ = INIT_Y_VAR_;
        yaw_var_ = INIT_YAW_VAR_;
        return true;
    }
    return false;
}

bool MCL::is_observation()
{
    if(ocd_.object_color_position.empty()) return false;
    return true;
}

double MCL::get_gaussian(double mu,double sigma)
{
    std::normal_distribution<> n_dist(mu,sigma);
    return n_dist(engine_);
}

double MCL::get_angle_diff(double a,double b)
{
    double a_angle = std::atan2(std::sin(a),std::cos(a));
    double b_angle = std::atan2(std::sin(b),std::cos(b));

    double d1 = a_angle-b_angle;
    double d2 = 2*M_PI - std::fabs(d1);

    if(d1 > 0) d2 *= -1;
    if(std::fabs(d1) < std::fabs(d2)) return d1;
    else return d2;
}

double MCL::get_weight(geometry_msgs::PoseStamped& pose) 
{
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double theta = tf2::getYaw(pose.pose.orientation);
    double weight = 0.0;
    for(const auto &p : ocd_.object_color_position){
        double distance = std::sqrt(p.x*p.x + p.z*p.z);
        double angle = std::atan2(p.z,p.x) - 0.5*M_PI;
        double tmp_robot_x = x + distance*std::cos(theta + angle);
        double tmp_robot_y = y + distance*std::sin(theta + angle);
        
        multi_localizer_msgs::RobotPoseStamped robot_pose;
        if(pose_subs_->get_pose(p.color,robot_pose)){
            double error_x = tmp_robot_x - robot_pose.pose.x;
            double error_y = tmp_robot_y - robot_pose.pose.y;
            double sigma = DISTANCE_NOISE_*distance;
            weight += robot_pose.pose.weight*weight_func(error_x,sigma)*weight_func(error_y,sigma);
        }
    }
    return weight;
}

double MCL::weight_func(double mu,double sigma) { return std::exp(-0.5*mu*mu/(sigma*sigma))/(std::sqrt(2.0*M_PI*sigma*sigma)); }

double MCL::get_particle_weight_sum()
{
    return std::accumulate(particles_.begin(),particles_.end(),0.0L,[](double l,Particle& p) { return l + p.weight_; });
}

double MCL::get_max_particle_weight()
{
    int max_index = std::distance(particles_.begin(),std::max_element(particles_.begin(),particles_.end(),[](const Particle& x,const Particle& y){ return x.weight_ < y.weight_; }));
    return particles_[max_index].weight_;
}

void MCL::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        if(is_start()){
            if(is_dense() || is_spread()){
                double tmp_x = estimated_pose_.pose.position.x;
                double tmp_y = estimated_pose_.pose.position.y;
                double tmp_yaw = tf2::getYaw(estimated_pose_.pose.orientation);
                set_particles(tmp_x,tmp_y,tmp_yaw,INIT_X_VAR_,INIT_Y_VAR_,INIT_YAW_VAR_);
            }
            motion_update();
            observation_update();
            if(is_update_){
                resample_particles();
                sort_by_particle_weight();
                is_update_ = false;
            }
            calc_estimated_pose();
            calc_variance();
            publish_particle_poses();
            publish_estimated_pose();
            if(IS_TF_) publish_tf();
        }
        has_received_odom_ = false;
        previous_odom_ = current_odom_;
        ros::spinOnce();
        rate.sleep();
    }
}