#include "object_localizer/mcl.h"

using namespace object_localizer;

MCL::MCL() :
    private_nh_("~"), start_time_(ros::Time::now()), engine_(seed_()),
    database_(new Database(nh_,private_nh_)),
    dynamic_objects_(new DynamicObjects(private_nh_)),
    odom_frame_id_(std::string("")),
    has_received_map_(false), has_received_odom_(false), is_update_(false),
    x_var_(0.0), y_var_(0.0), yaw_var_(0.0),
    weight_average_(0.0), weight_slow_(0.0), weight_fast_(0.0),
    distance_sum_(0.0), angle_sum_(0.0)
{
    private_nh_.param("MAP_FRAME_ID",MAP_FRAME_ID_,{std::string("map")});
    private_nh_.param("BASE_LINK_FRAME_ID",BASE_LINK_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("HZ",HZ_,{10});

    private_nh_.param("NUM_OF_PARTICLES",NUM_OF_PARTICLES_,{2000});
    private_nh_.param("INIT_X",INIT_X_,{7.0});
    private_nh_.param("INIT_Y",INIT_Y_,{0.0});
    private_nh_.param("INIT_YAW",INIT_YAW_,{1.5708});
    private_nh_.param("INIT_X_VAR",INIT_X_VAR_,{0.5});
    private_nh_.param("INIT_Y_VAR",INIT_Y_VAR_,{0.5});
    private_nh_.param("INIT_YAW_VAR",INIT_YAW_VAR_,{0.5});
    private_nh_.param("UPPER_X_VAR_TH",UPPER_X_VAR_TH_,{0.55});
    private_nh_.param("UPPER_Y_VAR_TH",UPPER_Y_VAR_TH_,{0.55});
    private_nh_.param("UPPER_YAW_VAR_TH",UPPER_YAW_VAR_TH_,{0.55});
    private_nh_.param("LOWER_X_VAR_TH",LOWER_X_VAR_TH_,{0.15});
    private_nh_.param("LOWER_Y_VAR_TH",LOWER_Y_VAR_TH_,{0.15});
    private_nh_.param("LOWER_YAW_VAR_TH",LOWER_YAW_VAR_TH_,{0.10});
    private_nh_.param("ALPHA_1",ALPHA_1_,{0.1});
    private_nh_.param("ALPHA_2",ALPHA_2_,{0.1});
    private_nh_.param("ALPHA_3",ALPHA_3_,{0.1});
    private_nh_.param("ALPHA_4",ALPHA_4_,{0.1});
    private_nh_.param("ALPHA_SLOW",ALPHA_SLOW_,{0.001});
    private_nh_.param("ALPHA_FAST",ALPHA_FAST_,{0.1});
    private_nh_.param("DISTANCE_TH",DISTANCE_TH_,{0.15});
    private_nh_.param("ANGLE_TH",ANGLE_TH_,{0.15});
    private_nh_.param("SELECTION_RATIO",SELECTION_RATIO_,{0.2});

    // observation parameters
    private_nh_.param("PROBABILITY_TH",PROBABILITY_TH_,{0.8});
    private_nh_.param("VISIBLE_LOWER_DISTANCE",VISIBLE_LOWER_DISTANCE_,{5.0});
    private_nh_.param("VISIBLE_UPPER_DISTANCE",VISIBLE_UPPER_DISTANCE_,{0.3});
    private_nh_.param("ANGLE_OF_VIEW",ANGLE_OF_VIEW_,{86.0/180.0*M_PI});
    private_nh_.param("DISTANCE_NOISE",DISTANCE_NOISE_,{0.1});
    private_nh_.param("ANGLE_NOISE",ANGLE_NOISE_,{0.1});

    map_sub_ = nh_.subscribe("map_in",1,&MCL::map_callback,this);
    odom_sub_ = nh_.subscribe("odom_in",1,&MCL::odom_callback,this);
    ops_sub_ = nh_.subscribe("ops_in",1,&MCL::ops_callback,this);

    private_nh_.param("IS_RECORD",IS_RECORD_,{false});
    if(IS_RECORD_){
        recorder_ = new Recorder(private_nh_);
        pose_sub_ = nh_.subscribe("pose_in",1,&MCL::pose_callback,this);
    }

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_out",1);
    poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("poses_out",1);

    if(IS_TF_){
        buffer_.reset(new tf2_ros::Buffer);
        listener_.reset(new tf2_ros::TransformListener(*buffer_));
        broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    }

    init();
}

MCL::~MCL()
{
    if(IS_RECORD_) recorder_->save_csv();
}

void MCL::map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if(has_received_map_) return;
    static_map_ = *msg;
    set_particles(INIT_X_,INIT_Y_,INIT_YAW_,INIT_X_VAR_,INIT_Y_VAR_,INIT_YAW_VAR_);
    has_received_map_ = true;
}

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
    odom_frame_id_ = msg->header.frame_id;
    current_odom_.header.stamp = msg->header.stamp;
    has_received_odom_ = true;
}

void MCL::ops_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
    filter_ops_msg(*msg,ops_);
}

void MCL::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    ref_pose_ = *msg;
}

void MCL::init()
{
    database_->load_init_objects();
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
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
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
    double dyaw = get_angle_diff(tf2::getYaw(current_odom_.pose.orientation),
                                 tf2::getYaw(previous_odom_.pose.orientation));

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
    if(ops_.object_position.empty()) return;
    if(IS_RECORD_){
        double time = (ros::Time::now() - start_time_).toSec();
        for(const auto &op : ops_.object_position){
            recorder_->add_observation(time,op.Class);
        }
    }
    for(auto &p : particles_) p.weight_ = get_weight(p.pose_);
    normalize_particles_weight();
    calc_weight_params();
}

void MCL::normalize_particles_weight()
{
    double weight_sum = get_particle_weight_sum();
    weight_average_ = weight_sum/NUM_OF_PARTICLES_;
    for(auto &p : particles_) p.weight_ /= weight_sum;
}

void MCL::calc_weight_params()
{
    if(weight_average_ ==  0 || std::isnan(weight_average_)){
        weight_average_ = 1 / (double)NUM_OF_PARTICLES_;
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
    int index = (int)(ur_dist(engine_)*NUM_OF_PARTICLES_);
    double beta = 0.0;
    double mv = get_max_particle_weight();
    std::vector<Particle> tmp_particles;
    for(int i = 0; i < (int)particles_.size(); i++){
        if(weight < ur_dist(engine_)){
            beta += 2.0*ur_dist(engine_)*mv;
            while(beta > particles_[index].weight_){
                beta -= particles_[index].weight_;
                index = (index + 1)%NUM_OF_PARTICLES_;
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
    int num_of_choices = (int)(SELECTION_RATIO_*NUM_OF_PARTICLES_);
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
    x_average /= (double)NUM_OF_PARTICLES_;
    y_average /= (double)NUM_OF_PARTICLES_;
    yaw_average /= (double)NUM_OF_PARTICLES_;

    double x_rss = 0.0;
    double y_rss = 0.0;
    double yaw_rss = 0.0;
    for(const auto &p : particles_){
        x_rss += std::pow(p.pose_.pose.position.x - x_average,2);
        y_rss += std::pow(p.pose_.pose.position.y - y_average,2);
        yaw_rss += std::pow(tf2::getYaw(p.pose_.pose.orientation) - yaw_average,2);
    }
    x_var_ = std::sqrt(x_rss/(double)NUM_OF_PARTICLES_);
    y_var_ = std::sqrt(y_rss/(double)NUM_OF_PARTICLES_);
    yaw_var_ = std::sqrt(yaw_rss/(double)NUM_OF_PARTICLES_);
}

void MCL::filter_ops_msg(object_detector_msgs::ObjectPositions input_ops,
                                           object_detector_msgs::ObjectPositions& output_ops)
{
	output_ops.header = input_ops.header;
    output_ops.object_position.clear();

    auto is_visible_range = [this](object_detector_msgs::ObjectPosition op) -> bool
    {
        if(dynamic_objects_->is_included(op.Class)) return false;
        if(op.Class == "fire_extinguisher") return false;
        if(op.Class == "chair") return false;
        if(op.Class == "table") return false;
        if(op.probability <= PROBABILITY_TH_) return false;

        double r_vertex_x = std::cos(0.5*(M_PI - ANGLE_OF_VIEW_));
        double r_vertex_y = std::sin(0.5*(M_PI - ANGLE_OF_VIEW_));
        double l_vertex_x = std::cos(0.5*(M_PI + ANGLE_OF_VIEW_));
        double l_vertex_y = std::sin(0.5*(M_PI + ANGLE_OF_VIEW_));

        double dist = std::sqrt(op.x*op.x + op.z*op.z);
        if(VISIBLE_LOWER_DISTANCE_ < dist &&
           dist < VISIBLE_UPPER_DISTANCE_){
            double x = op.x;
            double y = op.z;
            if(r_vertex_x*y - x*r_vertex_y >= 0 && l_vertex_x*y - x*l_vertex_y <= 0) return true;
        }
        return false;
    };

    for(const auto &inp_op : input_ops.object_position){
        if(is_visible_range(inp_op)) output_ops.object_position.emplace_back(inp_op);
    }
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

void MCL::publish_objects_msg()
{
    double weight = get_max_particle_weight();
    std::cout << "weight: " << weight << std::endl;
    ros::Time now_time = ros::Time::now();
    multi_robot_msgs::ObjectsData data;
    data.header.stamp = now_time;
    data.credibility = weight;
    for(const auto &op: ops_.object_position){
        multi_robot_msgs::ObjectData od;
        double distance = std::sqrt(op.x*op.x + op.z*op.z);
        double angle = std::atan2(op.z,op.x) - 0.5*M_PI;
        double estimated_yaw = tf2::getYaw(estimated_pose_.pose.orientation);

        od.name = op.Class;
        od.time = (now_time - start_time_).toSec();
        od.x = estimated_pose_.pose.position.x + distance*std::cos(estimated_yaw + angle);
        od.y = estimated_pose_.pose.position.y + distance*std::sin(estimated_yaw + angle);
        data.objects.emplace_back(od);
    }
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
        buffer_->transform(robot_to_map_pose,odom_to_map_pose,odom_frame_id_);
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
    map_to_odom_transform.child_frame_id = odom_frame_id_;
    tf2::convert(odom_to_map.inverse(),map_to_odom_transform.transform);
    broadcaster_->sendTransform(map_to_odom_transform);
}

void MCL::record_pose()
{
    double time = (ros::Time::now() - start_time_).toSec();
    recorder_->add_trajectory(time,
                              estimated_pose_.pose.position.x,
                              estimated_pose_.pose.position.y,
                              tf2::getYaw(estimated_pose_.pose.orientation),
                              ref_pose_.pose.pose.position.x,
                              ref_pose_.pose.pose.position.y,
                              tf2::getYaw(ref_pose_.pose.pose.orientation));
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

std::string MCL::get_date()
{
    time_t t = time(nullptr);
    const tm* localTime = localtime(&t);
    std::stringstream s;
    s << localTime->tm_year + 1900;
    s << std::setw(2) << std::setfill('0') << localTime->tm_mon + 1;
    s << std::setw(2) << std::setfill('0') << localTime->tm_mday;
    s << std::setw(2) << std::setfill('0') << localTime->tm_hour;
    s << std::setw(2) << std::setfill('0') << localTime->tm_min;
    s << std::setw(2) << std::setfill('0') << localTime->tm_sec;

    return s.str();
}

bool MCL::is_start()
{
    if(has_received_map_ && has_received_odom_) return true;
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
    double yaw = tf2::getYaw(pose.pose.orientation);
    double weight = 0.0;
    for(const auto & op : ops_.object_position){
        double distance = std::sqrt(op.x*op.x + op.z*op.z);
        double angle = std::atan2(op.z,op.x) - 0.5*M_PI;

        double sim;
        double tmp_yaw = tf2::getYaw(estimated_pose_.pose.orientation);
        double tmp_x = estimated_pose_.pose.position.x + distance*std::cos(tmp_yaw + angle);
        double tmp_y = estimated_pose_.pose.position.y + distance*std::sin(tmp_yaw + angle);
        Object nearest_object = database_->get_nearest_object(op.Class,tmp_x,tmp_y,sim);
        if(sim > 0.0){
            auto weight_func = [](double mu,double sigma) -> double
            {
                return std::exp(-0.5*mu*mu/(sigma*sigma))/(std::sqrt(2.0*M_PI*sigma*sigma));
            };
            double hat_x = x + distance*std::cos(yaw + angle);
            double hat_y = y + distance*std::sin(yaw + angle);
            double error_x = hat_x - nearest_object.x;
            double error_y = hat_y - nearest_object.y;
            double sigma = DISTANCE_NOISE_*distance;
            weight += weight_func(error_x,sigma)*weight_func(error_y,sigma);
        }
    }
    return weight;
}

double MCL::get_particle_weight_sum()
{
    return std::accumulate(particles_.begin(),particles_.end(),0.0L,[](double l,Particle& p) { return l + p.weight_; });
}

double MCL::get_max_particle_weight()
{
    int max_index = std::distance(particles_.begin(),std::max_element(particles_.begin(),particles_.end(),[](const Particle& x,const Particle& y){ return x.weight_ < y.weight_; }));
    return particles_[max_index].weight_;
}

MCL::Particle::Particle(MCL* mcl) :
    mcl_(mcl)
{
    mcl_->init_pose(pose_,0.0,0.0,0.0);
    weight_ = 1.0/(double)mcl_->NUM_OF_PARTICLES_;
}

void MCL::Particle::set_pose(double x,double y,double yaw,double x_var,double y_var,double yaw_var)
{
    double tmp_x = mcl_->get_gaussian(x,x_var);
    double tmp_y = mcl_->get_gaussian(y,y_var);
    double tmp_yaw = mcl_->get_gaussian(yaw,yaw_var);
    mcl_->set_pose(pose_,tmp_x,tmp_y,tmp_yaw);
}

void MCL::Particle::move(double dx,double dy,double dyaw)
{
    double yaw = tf2::getYaw(pose_.pose.orientation);

    double delta_trans = std::sqrt(dx*dx + dy*dy);
    double delta_rot1;
    if(delta_trans < 1e-2) delta_rot1 = 0.0;
    else delta_rot1 = dyaw;
    double delta_rot2 = mcl_->get_angle_diff(dyaw,delta_rot1);

    double delta_rot1_noise = std::min(std::fabs(mcl_->get_angle_diff(delta_rot1,0.0)),
                                       std::fabs(mcl_->get_angle_diff(delta_rot1,M_PI)));
    double delta_rot2_noise = std::min(std::fabs(mcl_->get_angle_diff(delta_rot2,0.0)),
                                       std::fabs(mcl_->get_angle_diff(delta_rot2,M_PI)));

    double rot1_sigma = mcl_->ALPHA_1_*delta_rot1_noise*delta_rot1_noise -
                        mcl_->ALPHA_2_*delta_trans*delta_trans;
    double rot2_sigma = mcl_->ALPHA_1_*delta_rot2_noise*delta_rot2_noise -
                        mcl_->ALPHA_2_*delta_trans*delta_trans;
    double trans_sigma = mcl_->ALPHA_3_*delta_trans*delta_trans +
                         mcl_->ALPHA_4_*delta_rot1_noise*delta_rot1_noise +
                         mcl_->ALPHA_4_*delta_rot2_noise*delta_rot2_noise;

    double delta_rot1_hat = mcl_->get_angle_diff(delta_rot1,mcl_->get_gaussian(0.0,rot1_sigma));
    double delta_rot2_hat = mcl_->get_angle_diff(delta_rot2,mcl_->get_gaussian(0.0,rot2_sigma));
    double delta_trans_hat = delta_trans - mcl_->get_gaussian(0.0,trans_sigma);

    double tmp_x = pose_.pose.position.x + delta_trans_hat*std::cos(yaw + delta_rot1_hat);
    double tmp_y = pose_.pose.position.y + delta_trans_hat*std::sin(yaw + delta_rot1_hat);
    double tmp_yaw = yaw + delta_rot1_hat + delta_rot2_hat;
    mcl_->set_pose(pose_,tmp_x,tmp_y,tmp_yaw);
}

void MCL::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        if(is_start()){
            database_->all_objects_are_not_observed();

            if(is_dense() /*|| is_spread()*/ ){
                double tmp_x = estimated_pose_.pose.position.x;
                double tmp_y = estimated_pose_.pose.position.y;
                double tmp_yaw = tf2::getYaw(estimated_pose_.pose.orientation);
                set_particles(tmp_x,tmp_y,tmp_yaw,INIT_X_VAR_,INIT_Y_VAR_,INIT_YAW_VAR_);
            }

            motion_update();
            if(!ops_.object_position.empty()){
                observation_update();
                if(is_update_){
                    resample_particles();
                    sort_by_particle_weight();
                    // calc_estimated_pose();
                    // calc_variance();
                    is_update_ = false;
                }
            }
            calc_estimated_pose();
            calc_variance();
            publish_particle_poses();
            publish_estimated_pose();
            publish_objects_msg();
            if(IS_RECORD_) record_pose();
            if(IS_TF_) publish_tf();
        }
        database_->publish_markers();
        has_received_odom_ = false;
        previous_odom_ = current_odom_;
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"mcl");
    object_localizer::MCL mcl;
    mcl.process();
    return 0;
}
