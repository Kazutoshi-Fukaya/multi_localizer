#include "mcl/mcl.h"

using namespace mcl;

MCL::MCL() :
    start_time_(ros::Time::now()),
    has_received_map_(false)
{
    // mcl base param
    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("PARTICLES_NUM",PARTICLES_NUM,{2000});
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

    // mcl params
    private_nh_.param("ROBOT_NAME",ROBOT_NAME_,{std::string("")});
    private_nh_.param("RANGE_STEP",RANGE_STEP_,{5});
    private_nh_.param("MAX_RANGE",MAX_RANGE_,{25});
    private_nh_.param("HIT_COV",HIT_COV_,{1.0});
    private_nh_.param("LAMBDA_SHORT",LAMBDA_SHORT_,{0.1});
    private_nh_.param("Z_HIT",Z_HIT_,{0.95});
    private_nh_.param("Z_SHORT",Z_SHORT_,{0.1});
    private_nh_.param("Z_MAX",Z_MAX_,{0.05});
    private_nh_.param("Z_RAND",Z_RAND_,{0.05});

    map_sub_ = nh_.subscribe("map_in",1,&MCL::map_callback,this);
    lsr_sub_ = nh_.subscribe("lsr_in",1,&MCL::lsr_callback,this);

    private_nh_.param("PUBLISH_ROBOT_POSE",PUBLISH_ROBOT_POSE_,{false});
    if(PUBLISH_ROBOT_POSE_){
        robot_pose_pub_ = nh_.advertise<multi_localizer_msgs::RobotPoseStamped>("robot_pose_out",1);
    }

    private_nh_.param("PUBLISH_OBJECTS_DATA",PUBLISH_OBJECTS_DATA_,{false});
    if(PUBLISH_OBJECTS_DATA_){
        od_sub_ = nh_.subscribe("od_in",1,&MCL::od_callback,this);
        data_pub_ = nh_.advertise<multi_localizer_msgs::ObjectsData>("data_out",1);
    }

    init();
}

MCL::~MCL() {}

void MCL::map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if(has_received_map_) return;
    static_map_ = *msg;
    has_received_map_ = true;
}

void MCL::lsr_callback(const sensor_msgs::LaserScanConstPtr& msg) { lsr_ = *msg; }

void MCL::od_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
    if(!PUBLISH_OBJECTS_DATA_ || msg->object_position.empty()) return;

    ros::Time now_time = ros::Time::now();
    double time = (now_time - start_time_).toSec();
    double x = estimated_pose_.pose.position.x;
    double y = estimated_pose_.pose.position.y;
    double yaw = tf2::getYaw(estimated_pose_.pose.orientation);

    multi_localizer_msgs::ObjectsData objects;
    objects.header.stamp = now_time;
    objects.header.frame_id = MAP_FRAME_ID_;
    objects.pose.name = ROBOT_NAME_;
    objects.pose.weight = 1.0; 
    objects.pose.x = x;
    objects.pose.y = y;
    objects.pose.theta = yaw;
    for(const auto &p : msg->object_position){
        if(p.Class != "roomba"){
            double dist = std::sqrt(p.x*p.x + p.z*p.z);
            double angle = std::atan2(p.z,p.x) - 0.5*M_PI;
            multi_localizer_msgs::ObjectData object;
            object.name = p.Class;
            object.time = time;
            object.credibility = 1.0*p.probability;
            object.x = x + dist*std::cos(yaw + angle);
            object.y = y + dist*std::sin(yaw + angle);
            objects.data.emplace_back(object);
        }
    }
    data_pub_.publish(objects);
}

void MCL::observation_update()
{
    for(auto &p : particles_) p.weight_ = get_weight(p.pose_);
    normalize_particles_weight();
    calc_weight_params();
}

void MCL::publish_tf()
{
    tf2::Quaternion q;
    q.setRPY(0.0,0.0,tf2::getYaw(estimated_pose_.pose.orientation));
    tf2::Transform map_to_robot(q,tf2::Vector3(estimated_pose_.pose.position.x,estimated_pose_.pose.position.y,0.0));

    geometry_msgs::PoseStamped robot_to_map_pose;
    robot_to_map_pose.header.frame_id = BASE_LINK_FRAME_ID_;
    robot_to_map_pose.header.stamp = lsr_.header.stamp;
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
    map_to_odom_transform.header.stamp = lsr_.header.stamp;
    map_to_odom_transform.header.frame_id = MAP_FRAME_ID_;
    map_to_odom_transform.child_frame_id = current_odom_.header.frame_id;
    tf2::convert(odom_to_map.inverse(),map_to_odom_transform.transform);
    broadcaster_->sendTransform(map_to_odom_transform);
}

bool MCL::is_start()
{
    if(has_received_map_ && has_received_odom_ && !lsr_.ranges.empty()) return true;
    return false;
}

double MCL::get_weight(geometry_msgs::PoseStamped& pose)
{
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double yaw = tf2::getYaw(pose.pose.orientation);
    double weight = 0.0;
    for(int i = 0; i < (int)lsr_.ranges.size(); i += RANGE_STEP_){
        double angle = lsr_.angle_min + i*lsr_.angle_increment;
        double dist = get_dist_to_wall(x,y,yaw + angle);
        double range_diff = lsr_.ranges[i] - dist;

        if(lsr_.ranges[i] < MAX_RANGE_){
            weight += Z_HIT_*std::exp(-range_diff*range_diff)/(2*HIT_COV_*HIT_COV_);
        }
        if(range_diff < 0.0){
            weight += Z_SHORT_*LAMBDA_SHORT_*std::exp(-LAMBDA_SHORT_*lsr_.ranges[i]);
        }
        if(lsr_.ranges[i] >= MAX_RANGE_){
            weight += Z_MAX_;
        }
        if(lsr_.ranges[i] < MAX_RANGE_){
            weight += Z_RAND_/MAX_RANGE_;
        }
    }
    return weight;
}

void MCL::publish_robot_pose()
{
    multi_localizer_msgs::RobotPoseStamped robot_pose;
    robot_pose.header = estimated_pose_.header;
    robot_pose.pose.name = ROBOT_NAME_;
    robot_pose.pose.weight = 1.0;
    robot_pose.pose.x = estimated_pose_.pose.position.x;
    robot_pose.pose.y = estimated_pose_.pose.position.y;
    robot_pose.pose.theta = tf2::getYaw(estimated_pose_.pose.orientation);
    robot_pose_pub_.publish(robot_pose);
}

double MCL::get_dist_to_wall(double x,double y,double yaw)
{
    int cx_0 = (x - static_map_.info.origin.position.x)/static_map_.info.resolution;
    int cy_0 = (y - static_map_.info.origin.position.y)/static_map_.info.resolution;

    int cx_1 = (x + MAX_RANGE_*std::cos(yaw) - static_map_.info.origin.position.x)/static_map_.info.resolution;
    int cy_1 = (y + MAX_RANGE_*std::sin(yaw) - static_map_.info.origin.position.y)/static_map_.info.resolution;

    bool judge = false;
    if(std::fabs(cx_1 - cx_0) < std::fabs(cy_1 - cy_0)) judge = true;

    if(judge){
        int tmp;
        tmp  = cx_1;
        cx_1 = cy_1;
        cy_1 = tmp;

        tmp  = cx_0;
        cx_0 = cy_0;
        cy_0 = tmp;
    }

    int dx = std::fabs(cx_1 - cx_0);
    int dy = std::fabs(cy_1 - cy_0);

    int cx = cx_0;
    int cy = cy_0;
    int error = 0;

    int xstep;
    if(cx_1 > cx_0) xstep = 1;
    else xstep = -1;

    int ystep;
    if(cy_1 > cy_0) ystep = 1;
    else ystep = -1;

    if(judge){
        if(cy < 0 || cy > (int)static_map_.info.width ||
           cx < 0 || cx > (int)static_map_.info.height ||
           static_map_.data[cx*(int)static_map_.info.width + cy] != 0){
            return std::sqrt(std::pow((cx - cx_0),2) + std::pow((cy - cy_0),2))*static_map_.info.resolution;
        }
    }
    else{
        if(cx < 0 || cx > (int)static_map_.info.width ||
           cy < 0 || cy > (int)static_map_.info.height ||
           static_map_.data[cy*(int)static_map_.info.width + cx] != 0){
            return std::sqrt(std::pow((cx - cx_0),2) + std::pow((cy - cy_0),2))*static_map_.info.resolution;
        }
    }

    while(cx != (cx_1 + xstep)){
        cx += xstep;
        error += dy;
        if(2*error >= dx){
            cy += ystep;
            error -= dx;
        }
        if(judge){
            if(cy < 0 || cy > (int)static_map_.info.width ||
               cx < 0 || cx > (int)static_map_.info.height ||
               static_map_.data[cx*(int)static_map_.info.width + cy] != 0){
                return std::sqrt(std::pow((cx - cx_0),2) + std::pow((cy - cy_0),2))*static_map_.info.resolution;
            }
        }
        else{
            if(cx < 0 || cx > (int)static_map_.info.width ||
               cy < 0 || cy > (int)static_map_.info.height ||
               static_map_.data[cy*(int)static_map_.info.width + cx] != 0){
                return std::sqrt(std::pow((cx - cx_0),2) + std::pow((cy - cy_0),2))*static_map_.info.resolution;
            }
        }
    }
    return MAX_RANGE_;
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
                calc_estimated_pose();
                calc_variance();
                is_update_ = false;
            }
            publish_particle_poses();
            publish_estimated_pose();
            if(PUBLISH_ROBOT_POSE_) publish_robot_pose();
            publish_tf();
        }
        has_received_odom_ = false;
        previous_odom_ = current_odom_;
        ros::spinOnce();
        rate.sleep();
    }
}