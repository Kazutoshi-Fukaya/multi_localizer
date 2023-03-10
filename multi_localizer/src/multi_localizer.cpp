#include "multi_localizer/multi_localizer.h"

using namespace multi_localizer;

MultiLocalizer::MultiLocalizer() :
    start_time_(ros::Time::now()), has_got_weight_(false)
{
    // MCL Base params
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
    private_nh_.param("ALPHA_1",ALPHA_1_,{0.1});
    private_nh_.param("ALPHA_2",ALPHA_2_,{0.1});
    private_nh_.param("ALPHA_3",ALPHA_3_,{0.1});
    private_nh_.param("ALPHA_4",ALPHA_4_,{0.1});
    private_nh_.param("ALPHA_SLOW",ALPHA_SLOW_,{0.001});
    private_nh_.param("ALPHA_FAST",ALPHA_FAST_,{0.1});
    private_nh_.param("DISTANCE_TH",DISTANCE_TH_,{0.15});
    private_nh_.param("ANGLE_TH",ANGLE_TH_,{0.15});
    private_nh_.param("SELECTION_RATIO",SELECTION_RATIO_,{0.2});

    // place recognition params
    private_nh_.param("USE_PLACE_RECOGNITION",USE_PLACE_RECOGNITION_,{true});
    if(USE_PLACE_RECOGNITION_){
        std::cout << "USE_PLACE_RECOGNITION" << std::endl;
        private_nh_.param("ALPHA",ALPHA_,{0.7});
        private_nh_.param("ERROR_TH",ERROR_TH_,{1.5});
        private_nh_.param("SCORE_TH",SCORE_TH_,{0.6});

        pr_pose_sub_ = nh_.subscribe("pr_pose_in",1,&MultiLocalizer::pr_callback,this);
    }

    // object detection params
    private_nh_.param("USE_OBJECT_DETECTION",USE_OBJECT_DETECTION_,{true});
    if(USE_OBJECT_DETECTION_){
        std::cout << "USE_OBJECT_DETECTION" << std::endl;
        private_nh_.param("BETA",BETA_,{0.2});
        private_nh_.param("PROBABILITY_TH",PROBABILITY_TH_,{0.8});
        private_nh_.param("VISIBLE_LOWER_DISTANCE",VISIBLE_LOWER_DISTANCE_,{5.0});
        private_nh_.param("VISIBLE_UPPER_DISTANCE",VISIBLE_UPPER_DISTANCE_,{0.3});
        private_nh_.param("ANGLE_OF_VIEW",ANGLE_OF_VIEW_,{86.0/180.0*M_PI});
        private_nh_.param("SIM_TH",SIM_TH_,{0.0});

        robot_name_list_ = new RobotNameList(private_nh_);
        object_map_ = new ObjectMap();
        load_object_params(object_map_);
        load_init_objects(object_map_);

        od_sub_ = nh_.subscribe("od_in",1,&MultiLocalizer::od_callback,this);

        private_nh_.param("PUBLISH_OBJECTS_DATA",PUBLISH_OBJECTS_DATA_,{false});
        if(PUBLISH_OBJECTS_DATA_){
            private_nh_.param("ROBOT_NAME",ROBOT_NAME_,{std::string("roomba")});
            data_pub_ = nh_.advertise<multi_localizer_msgs::ObjectsData>("data_out",1);
        }

        private_nh_.param("PUBLISH_MARKERS",PUBLISH_MARKERS_,{false});
        if(PUBLISH_MARKERS_){
            markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers_out",1);
        }

        private_nh_.param("SUBSCRIBE_OBJECT_DATA",SUBSCRIBE_OBJECT_DATA_,{false});
        if(SUBSCRIBE_OBJECT_DATA_){
            object_map_sub_ = nh_.subscribe("object_map_in",1,&MultiLocalizer::object_map_callback,this);
        }
    }

    // mutual recognition params
    private_nh_.param("USE_MUTUAL_RECOGNITION",USE_MUTUAL_RECOGNITION_,{false});
    if(USE_MUTUAL_RECOGNITION_){
        std::cout << "USE_MUTUAL_RECOGNITION" << std::endl;
        pose_subscribers_ = new PoseSubscribers(nh_,private_nh_);
        ocd_sub_ = nh_.subscribe("ocd_in",1,&MultiLocalizer::ocd_callback,this);
    }

    // 
    if(USE_OBJECT_DETECTION_ || USE_MUTUAL_RECOGNITION_){
        private_nh_.param("DISTANCE_NOISE",DISTANCE_NOISE_,{0.1});
    }

    init();
}

MultiLocalizer::~MultiLocalizer() {}

void MultiLocalizer::pr_callback(const place_recognition_msgs::PoseStampedConstPtr& msg) { pr_pose_ = *msg; }

void MultiLocalizer::od_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg) { filter_od_msg(*msg,od_); }

void MultiLocalizer::ocd_callback(const object_color_detector_msgs::ObjectColorPositionsConstPtr& msg) { ocd_ = *msg; }

void MultiLocalizer::object_map_callback(const multi_localizer_msgs::ObjectMapConstPtr& msg)
{
    // TO DO
}

void MultiLocalizer::observation_update()
{
    for(auto &p : particles_){
        p.weight_ = get_weight(p.pose_);
    }
    std::cout << "max weight: " << get_max_particle_weight() << std::endl;
    if(has_got_weight_){
        normalize_particles_weight();
        calc_weight_params();
    }
    has_got_weight_ = false;
}

void MultiLocalizer::publish_tf()
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

bool MultiLocalizer::is_start()
{
    if(has_received_odom_) return true;
    return false;
}

// no use
bool MultiLocalizer::is_observation() { return true; }

double MultiLocalizer::get_weight(geometry_msgs::PoseStamped& pose)
{
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double theta = tf2::getYaw(pose.pose.orientation);
    double weight = 0.0;

    // place recognition
    if(is_pr_observation()){
        double diff_x = x - pr_pose_.x;
        double diff_y = y - pr_pose_.y;
        double diff_theta = theta - pr_pose_.theta;
        double score = pr_pose_.score;

        // position
        double dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
        weight += ALPHA_*weight_func(dist,score);

        // theta
        weight += ALPHA_*weight_func(diff_theta,score);
        has_got_weight_ = true;
    }

    // object detection
    if(is_od_observation()){
        for(const auto & op : od_.object_position){
            double distance = std::sqrt(op.x*op.x + op.z*op.z);
            double angle = std::atan2(op.z,op.x) - 0.5*M_PI;

            double tmp_yaw = tf2::getYaw(estimated_pose_.pose.orientation);
            double tmp_x = estimated_pose_.pose.position.x + distance*std::cos(tmp_yaw + angle);
            double tmp_y = estimated_pose_.pose.position.y + distance*std::sin(tmp_yaw + angle);
            double sim;
            Object sim_object = object_map_->get_highly_similar_object(op.Class,tmp_x,tmp_y,sim);
            // std::cout << "Object(sim): " << op.Class << "(" << sim << ")" <<  std::endl;
            if(sim > SIM_TH_){
                double hat_x = x + distance*std::cos(theta + angle);
                double hat_y = y + distance*std::sin(theta + angle);
                double error_x = hat_x - sim_object.x;
                double error_y = hat_y - sim_object.y;
                double sigma = DISTANCE_NOISE_*distance;
                weight += BETA_*weight_func(error_x,sigma)*weight_func(error_y,sigma);
            }
        }
        has_got_weight_ = true;
    }

    // mutual recognition
    if(is_mr_observation()){
        for(const auto &p : ocd_.object_color_position){
            double distance = std::sqrt(p.x*p.x + p.z*p.z);
            double angle = std::atan2(p.z,p.x) - 0.5*M_PI;
            double tmp_robot_x = x + distance*std::cos(theta + angle);
            double tmp_robot_y = y + distance*std::sin(theta + angle);

            multi_localizer_msgs::RobotPoseStamped robot_pose;
            if(pose_subscribers_->get_pose(p.color,robot_pose)){
                double error_x = tmp_robot_x - robot_pose.pose.x;
                double error_y = tmp_robot_y - robot_pose.pose.y;
                double sigma = DISTANCE_NOISE_*distance;
                weight += robot_pose.pose.weight*weight_func(error_x,sigma)*weight_func(error_y,sigma);
            }
        }
        has_got_weight_ = true;
        // ocd_.object_color_position.clear();
    }
    return weight;
}

bool MultiLocalizer::is_pr_observation()
{
    if(!USE_PLACE_RECOGNITION_) return false;
    double diff_x = estimated_pose_.pose.position.x - pr_pose_.x;
    double diff_y = estimated_pose_.pose.position.y - pr_pose_.y;
    double dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
    if(dist < ERROR_TH_) return true;
    if(pr_pose_.score > SCORE_TH_){
        set_particles(pr_pose_.x,pr_pose_.y,pr_pose_.theta,INIT_X_VAR_,INIT_Y_VAR_,INIT_YAW_VAR_);
    }
    return false;
}

void MultiLocalizer::load_object_params(ObjectMap* object_map)
{
    object_map->clear();
    std::string yaml_file_name;
    private_nh_.param("YAML_FILE_NAME",yaml_file_name,{std::string("object_params")});
    XmlRpc::XmlRpcValue object_params;
    if(!private_nh_.getParam(yaml_file_name.c_str(),object_params)){
        ROS_WARN("Cloud not load %s", yaml_file_name.c_str());
        return;
    }

    ROS_ASSERT(object_params.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < object_params.size(); i++){
        if(!object_params[i]["name"].valid() ||
           !object_params[i]["condition"].valid() ||
           !object_params[i]["r"].valid() || !object_params[i]["g"].valid() || !object_params[i]["b"].valid()){
            ROS_WARN("%s is valid", yaml_file_name.c_str());
            return;
        }
        if(object_params[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
           object_params[i]["condition"].getType() == XmlRpc::XmlRpcValue::TypeString &&
           object_params[i]["r"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
           object_params[i]["g"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
           object_params[i]["b"].getType() == XmlRpc::XmlRpcValue::TypeDouble){
            std::string name = static_cast<std::string>(object_params[i]["name"]);
            std::string condition = static_cast<std::string>(object_params[i]["condition"]);
            double r = static_cast<double>(object_params[i]["r"]);
            double g = static_cast<double>(object_params[i]["g"]);
            double b = static_cast<double>(object_params[i]["b"]);
            ObjectParam* object_param (new ObjectParam(name,condition,Color(r,g,b)));
            Objects* objects (new Objects());
            object_map->insert(std::map<ObjectParam*,Objects*>::value_type(object_param,objects));
        }
    }
}

void MultiLocalizer::load_init_objects(ObjectMap* object_map)
{
    std::string init_objects_file_name;
    private_nh_.param("INIT_OBJECTS_FILE_NAME",init_objects_file_name,{std::string("")});
    std::ifstream ifs(init_objects_file_name);
    std::string line;
    while(std::getline(ifs,line)){
        std::vector<std::string> strvec = split(line,',');
        try{
            std::string name = static_cast<std::string>(strvec[0]);
            double time = static_cast<double>(std::stod(strvec[1]));
            double x = static_cast<double>(std::stod(strvec[2]));
            double y = static_cast<double>(std::stod(strvec[3]));
            object_map->add_init_object(name,Object(time,1.0,x,y));
        }
        catch(const std::invalid_argument& ex){
            ROS_ERROR("invalid: %s", ex.what());
        }
        catch(const std::out_of_range& ex){
            ROS_ERROR("out of range: %s", ex.what());
        }
    }
    ifs.close();
}

void MultiLocalizer::filter_od_msg(object_detector_msgs::ObjectPositions input_od,
                                   object_detector_msgs::ObjectPositions& output_od)
{
    output_od.header = input_od.header;
    output_od.object_position.clear();
    for(const auto &p : input_od.object_position){
        if(is_visible_range(p)) output_od.object_position.emplace_back(p);
    }
}

void MultiLocalizer::publish_objects_data()
{
    double tmp_weight = get_max_particle_weight();
    double tmp_x = estimated_pose_.pose.position.x;
    double tmp_y = estimated_pose_.pose.position.y;
    double tmp_yaw = tf2::getYaw(estimated_pose_.pose.orientation);

    ros::Time now_time = ros::Time::now();
    multi_localizer_msgs::ObjectsData objects;
    objects.header.stamp = now_time;
    objects.header.frame_id = MAP_FRAME_ID_;
    objects.pose.weight = tmp_weight;
    objects.pose.x = tmp_x;
    objects.pose.y = tmp_y;
    objects.pose.theta = tmp_yaw;
    for(const auto &p : od_.object_position){
        multi_localizer_msgs::ObjectData object;
        double distance = std::sqrt(p.x*p.x + p.z*p.z);
        double angle = std::atan2(p.z,p.x) - 0.5*M_PI;

        object.name = p.Class;
        object.time = (now_time - start_time_).toSec();
        object.credibility = tmp_weight;
        object.x = tmp_x + distance*std::cos(tmp_yaw + angle);
        object.y = tmp_y + distance*std::sin(tmp_yaw + angle);
        objects.data.emplace_back(object);
    }
    data_pub_.publish(objects);
    od_.object_position.clear();
}

void MultiLocalizer::publish_markers()
{
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    for(auto it = object_map_->begin(); it != object_map_->end(); it++){
        for(auto sit = it->second->begin(); sit != it->second->end(); sit++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = MAP_FRAME_ID_;
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.6;
            marker.id = marker_id;
            marker.ns = it->first->name;
            marker.pose.position.x = sit->x;
            marker.pose.position.y = sit->y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.color.r = it->first->color.r;
            marker.color.g = it->first->color.g;
            marker.color.b = it->first->color.b;
            if(sit->has_observed) marker.color.a = 1.0;
            else marker.color.a = 0.3;
            markers.markers.emplace_back(marker);
            marker_id++;
        }
    }
    markers_pub_.publish(markers);
}

bool MultiLocalizer::is_od_observation()
{
    if(!USE_OBJECT_DETECTION_) return false;
    if(od_.object_position.empty()) return false;
    return true;
}

bool MultiLocalizer::is_visible_range(object_detector_msgs::ObjectPosition op)
{
    if(robot_name_list_->is_included(op.Class)) return false;
    if(op.Class == "fire_extinguisher") return false;

    if(op.probability <= PROBABILITY_TH_) return false;

    double r_vertex_x = std::cos(0.5*(M_PI - ANGLE_OF_VIEW_));
    double r_vertex_y = std::sin(0.5*(M_PI - ANGLE_OF_VIEW_));
    double l_vertex_x = std::cos(0.5*(M_PI + ANGLE_OF_VIEW_));
    double l_vertex_y = std::sin(0.5*(M_PI + ANGLE_OF_VIEW_));

    double dist = std::sqrt(op.x*op.x + op.z*op.z);
    if(VISIBLE_LOWER_DISTANCE_ < dist && dist < VISIBLE_UPPER_DISTANCE_){
        double x = op.x;
        double y = op.z;
        if(r_vertex_x*y - x*r_vertex_y >= 0 && l_vertex_x*y - x*l_vertex_y <= 0) return true;
    }
    return false;
}

bool MultiLocalizer::is_mr_observation()
{
    if(!USE_MUTUAL_RECOGNITION_) return false;
    if(ocd_.object_color_position.empty()) return false;
    return true;
}


std::vector<std::string> MultiLocalizer::split(std::string& input,char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.push_back(field);
    return result;
}

double MultiLocalizer::weight_func(double mu,double sigma) { return std::exp(-0.5*mu*mu/(sigma*sigma))/(std::sqrt(2.0*M_PI*sigma*sigma)); }

void MultiLocalizer::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        if(is_start()){
            if(USE_OBJECT_DETECTION_){
                object_map_->all_objects_are_not_observed();
            }

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
            if(USE_OBJECT_DETECTION_ && PUBLISH_OBJECTS_DATA_) publish_objects_data();
            if(IS_TF_) publish_tf();
        }
        if(USE_OBJECT_DETECTION_ && PUBLISH_MARKERS_) publish_markers();
        has_received_odom_ = false;
        previous_odom_ = current_odom_;
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"multi_localizer");
    MultiLocalizer multi_localizer;
    multi_localizer.process();
    return 0;
}
