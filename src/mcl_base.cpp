#include "mcl_base/mcl_base.h"

multi_localizer::MCLBase::MCLBase() :
	private_nh_("~"), engine_(seed_()),
	has_received_odom_(false), is_update_(false),
	x_var_(0.0), y_var_(0.0), yaw_var_(0.0),
	weight_average_(0.0), weight_slow_(0.0), weight_fast_(0.0),
	distance_sum_(0.0), angle_sum_(0.0)
{
	private_nh_.param("MAP_FRAME_ID",MAP_FRAME_ID_,{std::string("map")});
	private_nh_.param("BASE_LINK_FRAME_ID",BASE_LINK_FRAME_ID_,{std::string("base_link")});

    odom_sub_ = nh_.subscribe("odom_in",1,&MCLBase::odom_callback,this);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_out",1);
    poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("poses_out",1);

    private_nh_.param("IS_TF",IS_TF_,{true});
    if(IS_TF_){
        buffer_.reset(new tf2_ros::Buffer);
        listener_.reset(new tf2_ros::TransformListener(*buffer_));
        broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    }
}

void multi_localizer::MCLBase::odom_callback(const nav_msgs::OdometryConstPtr& msg)
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

void multi_localizer::MCLBase::observation_update() {}

void multi_localizer::MCLBase::publish_tf() {}

bool multi_localizer::MCLBase::is_start() { return true; }

double multi_localizer::MCLBase::get_weight(geometry_msgs::PoseStamped& pose) { return 0.0; }

void multi_localizer::MCLBase::init()
{
	set_particles(INIT_X_,INIT_Y_,INIT_YAW_,INIT_X_VAR_,INIT_Y_VAR_,INIT_YAW_VAR_);
    init_pose(previous_odom_,0.0,0.0,0.0);
    init_pose(current_odom_,0.0,0.0,0.0);
    init_pose(estimated_pose_,INIT_X_,INIT_Y_,INIT_YAW_);
}

void multi_localizer::MCLBase::init_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw)
{
    pose.header.frame_id = MAP_FRAME_ID_;
    set_pose(pose,x,y,yaw);
}

void multi_localizer::MCLBase::set_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw)
{
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = get_quat_msg_from_yaw(yaw);
}

void multi_localizer::MCLBase::set_particles(double x,double y,double yaw,double x_var,double y_var,double yaw_var)
{
    std::vector<Particle> tmp_particles;
    for(int i = 0; i < PARTICLES_NUM; i++){
        Particle particle = generate_particle();
        particle.set_pose(x,y,yaw,x_var,y_var,yaw_var);
        tmp_particles.emplace_back(particle);
    }
    particles_ = tmp_particles;
}

void multi_localizer::MCLBase::motion_update()
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

void multi_localizer::MCLBase::normalize_particles_weight()
{
    double weight_sum = get_particle_weight_sum();
    weight_average_ = weight_sum/PARTICLES_NUM;
    for(auto &p : particles_) p.weight_ /= weight_sum;
}

void multi_localizer::MCLBase::calc_weight_params()
{
    if(weight_average_ ==  0 || std::isnan(weight_average_)){
        weight_average_ = 1 / (double)PARTICLES_NUM;
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

void multi_localizer::MCLBase::resample_particles()
{
    double weight;
    if((1 - weight_fast_/weight_slow_) > 0) weight = 1 - weight_fast_/weight_slow_;
    else weight = 0.0;

    std::uniform_real_distribution<> ur_dist(0.0,1.0);
    int index = (int)(ur_dist(engine_)*PARTICLES_NUM);
    double beta = 0.0;
    double mv = get_max_particle_weight();
    std::vector<Particle> tmp_particles;
    for(int i = 0; i < (int)particles_.size(); i++){
        if(weight < ur_dist(engine_)){
            beta += 2.0*ur_dist(engine_)*mv;
            while(beta > particles_[index].weight_){
                beta -= particles_[index].weight_;
                index = (index + 1)%PARTICLES_NUM;
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

void multi_localizer::MCLBase::sort_by_particle_weight()
{
    std::sort(particles_.begin(),particles_.end(),[](const Particle& x,const Particle& y){ return x.weight_ > y.weight_; });
}

void multi_localizer::MCLBase::calc_estimated_pose()
{
    double tmp_est_x = 0.0;
    double tmp_est_y = 0.0;
    int choices_num = (int)(SELECTION_RATIO_*PARTICLES_NUM);
    for(int i = 0; i < choices_num; i++){
        tmp_est_x += particles_[i].pose_.pose.position.x;
        tmp_est_y += particles_[i].pose_.pose.position.y;
    }
    tmp_est_x /= (double)choices_num;
    tmp_est_y /= (double)choices_num;
    double tmp_est_yaw = tf2::getYaw(particles_[0].pose_.pose.orientation);

    set_pose(estimated_pose_,tmp_est_x,tmp_est_y,tmp_est_yaw);
}

void multi_localizer::MCLBase::calc_variance()
{
    double x_average = 0.0;
    double y_average = 0.0;
    double yaw_average = 0.0;
    for(const auto &p : particles_){
        x_average += p.pose_.pose.position.x;
        y_average += p.pose_.pose.position.y;
        yaw_average += tf2::getYaw(p.pose_.pose.orientation);
    }
    x_average /= (double)PARTICLES_NUM;
    y_average /= (double)PARTICLES_NUM;
    yaw_average /= (double)PARTICLES_NUM;

    double x_rss = 0.0;
    double y_rss = 0.0;
    double yaw_rss = 0.0;
    for(const auto &p : particles_){
        x_rss += std::pow(p.pose_.pose.position.x - x_average,2);
        y_rss += std::pow(p.pose_.pose.position.y - y_average,2);
        yaw_rss += std::pow(tf2::getYaw(p.pose_.pose.orientation) - yaw_average,2);
    }
    x_var_ = std::sqrt(x_rss/(double)PARTICLES_NUM);
    y_var_ = std::sqrt(y_rss/(double)PARTICLES_NUM);
    yaw_var_ = std::sqrt(yaw_rss/(double)PARTICLES_NUM);
}

void multi_localizer::MCLBase::publish_particle_poses()
{
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = MAP_FRAME_ID_;
    poses.header.stamp = ros::Time::now();
    for(const auto &p : particles_) poses.poses.emplace_back(p.pose_.pose);
    poses_pub_.publish(poses);
}

void multi_localizer::MCLBase::publish_estimated_pose()
{
    estimated_pose_.header.stamp = ros::Time::now();
    pose_pub_.publish(estimated_pose_);
}

multi_localizer::MCLBase::Particle multi_localizer::MCLBase::generate_particle() { return multi_localizer::MCLBase::Particle(this); }

geometry_msgs::Quaternion multi_localizer::MCLBase::get_quat_msg_from_yaw(double yaw)
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

bool multi_localizer::MCLBase::is_dense()
{
    if(x_var_ < LOWER_X_VAR_TH_ || y_var_ < LOWER_Y_VAR_TH_ || yaw_var_ < LOWER_YAW_VAR_TH_){
        x_var_ = INIT_X_VAR_;
        y_var_ = INIT_Y_VAR_;
        yaw_var_ = INIT_YAW_VAR_;
        return true;
    }
    return false;
}

bool multi_localizer::MCLBase::is_spread()
{
	if(x_var_ > UPPER_X_VAR_TH_ || y_var_ > UPPER_Y_VAR_TH_ || yaw_var_ > UPPER_YAW_VAR_TH_){
		x_var_ = INIT_X_VAR_;
		y_var_ = INIT_Y_VAR_;
		yaw_var_ = INIT_YAW_VAR_;
		return true;		
	}
	return false;
}

double multi_localizer::MCLBase::get_gaussian(double mu,double sigma)
{
    std::normal_distribution<> n_dist(mu,sigma);
    return n_dist(engine_);
}

double multi_localizer::MCLBase::get_angle_diff(double a,double b)
{
    double a_angle = std::atan2(std::sin(a),std::cos(a));
    double b_angle = std::atan2(std::sin(b),std::cos(b));

    double d1 = a_angle-b_angle;
    double d2 = 2*M_PI - std::fabs(d1);

    if(d1 > 0) d2 *= -1;
    if(std::fabs(d1) < std::fabs(d2)) return d1;
    else return d2;
}

double multi_localizer::MCLBase::get_particle_weight_sum()
{
    return std::accumulate(particles_.begin(),particles_.end(),0.0L,[](double l,Particle& p) { return l + p.weight_; });
}

double multi_localizer::MCLBase::get_max_particle_weight()
{
    int max_index = std::distance(particles_.begin(),std::max_element(particles_.begin(),particles_.end(),[](const Particle& x,const Particle& y){ return x.weight_ < y.weight_; }));
    return particles_[max_index].weight_;
}
