#include "single_mcl/single_mcl.h"

multi_localizer::SingleMCL::SingleMCL() :
    start_time_(ros::Time::now()),
    has_received_map_(false)
{
    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("PARTICLES_NUM",PARTICLES_NUM,{2000});
    private_nh_.param("RANGE_STEP",RANGE_STEP_,{5});
    private_nh_.param("INIT_X",INIT_X_,{7.0});
    private_nh_.param("INIT_Y",INIT_Y_,{0.0});
    private_nh_.param("INIT_YAW",INIT_YAW_,{1.5708});
    private_nh_.param("INIT_X_VAR",INIT_X_VAR_,{0.5});
    private_nh_.param("INIT_Y_VAR",INIT_Y_VAR_,{0.5});
    private_nh_.param("INIT_YAW_VAR",INIT_YAW_VAR_,{0.5});
    private_nh_.param("MAX_RANGE",MAX_RANGE_,{25});
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
    private_nh_.param("HIT_COV",HIT_COV_,{1.0});
    private_nh_.param("LAMBDA_SHORT",LAMBDA_SHORT_,{0.1});
    private_nh_.param("Z_HIT",Z_HIT_,{0.95});
    private_nh_.param("Z_SHORT",Z_SHORT_,{0.1});
    private_nh_.param("Z_MAX",Z_MAX_,{0.05});
    private_nh_.param("Z_RAND",Z_RAND_,{0.05});
    private_nh_.param("DISTANCE_TH",DISTANCE_TH_,{0.15});
    private_nh_.param("ANGLE_TH",ANGLE_TH_,{0.15});
    private_nh_.param("SELECTION_RATIO",SELECTION_RATIO_,{0.2});

    map_sub_ = nh_.subscribe("map_in",1,&SingleMCL::map_callback,this);
    lsr_sub_ = nh_.subscribe("lsr_in",1,&SingleMCL::lsr_callback,this);

    // private_nh_.param("IS_RECORD",IS_RECORD_,{false});
    // private_nh_.param("PUBLISH_OBJ_DATA",PUBLISH_OBJ_DATA_,{false});
    // if(IS_RECORD_ || PUBLISH_OBJ_DATA_){
    //     obj_sub_ = nh_.subscribe("obj_in",1,&SingleMCL::obj_callback,this);
    //     if(IS_RECORD_){
    //         recorder_ = new Recorder(private_nh_);
    //         pose_sub_ = nh_.subscribe("pose_in",1,&SingleMCL::pose_callback,this);
    //     }
    //     if(PUBLISH_OBJ_DATA_){
    //         obj_pub_ = nh_.advertise<multi_robot_msgs::ObjectsData>("obj_out",1);
    //     }
    // }

    init();
}

multi_localizer::SingleMCL::~SingleMCL()
{
    // if(IS_RECORD_) recorder_->save_csv();
}

void multi_localizer::SingleMCL::map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if(has_received_map_) return;
    static_map_ = *msg;
    has_received_map_ = true;
}

void multi_localizer::SingleMCL::lsr_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    lsr_ = *msg;
}

// void multi_localizer::SingleMCL::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
// {
//     ref_pose_.header = msg->header;
//     ref_pose_.pose = msg->pose.pose;
// }

// void multi_localizer::SingleMCL::obj_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
// {
//     if(msg->object_position.empty()) return;

//     ros::Time now_time = ros::Time::now();
//     double time = (now_time - start_time_).toSec();

//     multi_robot_msgs::ObjectsData objects_data;
//     objects_data.credibility = 1.0; // temporary
//     objects_data.header.stamp = now_time;
//     objects_data.header.frame_id = MAP_FRAME_ID_;
//     objects_data.pose.name = "";    // temporary
//     objects_data.pose.weight = 1.0; // temporary
//     objects_data.pose.x = estimated_pose_.pose.position.x;
//     objects_data.pose.y = estimated_pose_.pose.position.y;
//     objects_data.pose.yaw = tf2::getYaw(estimated_pose_.pose.orientation);
//     if(IS_RECORD_ || PUBLISH_OBJ_DATA_){
//         for(const auto &op : msg->object_position){
//             // atode naosu
//             if(op.Class != "roomba"){
//                 if(IS_RECORD_) recorder_->add_observation(time,op.Class);
//                 if(PUBLISH_OBJ_DATA_){
//                     double dist = std::sqrt(op.x*op.x + op.z*op.z);
//                     double angle = std::atan2(op.z,op.x) - 0.5*M_PI;
//                     multi_robot_msgs::ObjectData data;
//                     data.name = op.Class;
//                     data.time = time;
//                     data.x = objects_data.pose.x + dist*std::cos(angle + objects_data.pose.yaw);
//                     data.y = objects_data.pose.y + dist*std::sin(angle + objects_data.pose.yaw);
//                     objects_data.objects.emplace_back(data);
//                 }
//             }
//         }
//     }
//     if(PUBLISH_OBJ_DATA_) obj_pub_.publish(objects_data);
// }

void multi_localizer::SingleMCL::observation_update()
{
    for(auto &p : particles_) p.weight_ = get_weight(p.pose_);
    normalize_particles_weight();
    calc_weight_params();
}

void multi_localizer::SingleMCL::publish_tf()
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

// void multi_localizer::SingleMCL::record_pose()
// {
//     double time = (ros::Time::now() - start_time_).toSec();
//     recorder_->add_trajectory(time,
//                               estimated_pose_.pose.position.x,
//                               estimated_pose_.pose.position.y,
//                               tf2::getYaw(estimated_pose_.pose.orientation),
//                               ref_pose_.pose.position.x,
//                               ref_pose_.pose.position.y,
//                               tf2::getYaw(ref_pose_.pose.orientation));
// }

bool multi_localizer::SingleMCL::is_start()
{
    if(has_received_map_ && has_received_odom_ && !lsr_.ranges.empty()) return true;
    return false;
}

double multi_localizer::SingleMCL::get_weight(geometry_msgs::PoseStamped& pose)
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

double multi_localizer::SingleMCL::get_dist_to_wall(double x,double y,double yaw)
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

void multi_localizer::SingleMCL::process()
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
            publish_tf();
            // if(IS_RECORD_) record_pose();
        }
        has_received_odom_ = false;
        previous_odom_ = current_odom_;
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"single_mcl");
    multi_localizer::SingleMCL single_mcl;
    single_mcl.process();
    return 0;
}
