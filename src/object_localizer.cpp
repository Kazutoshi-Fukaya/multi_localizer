#include "object_localizer/object_localizer.h"

multi_localizer::ObjectLocalizer::ObjectLocalizer() :
	start_time_(ros::Time::now()),
	robot_name_list_(new RobotNameList(private_nh_)),
	database_(new Database(nh_,private_nh_)),
	pose_subscribers_(new PoseSubscribers(nh_,private_nh_))
{
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
	private_nh_.param("PROBABILITY_TH",PROBABILITY_TH_,{0.8});
    private_nh_.param("VISIBLE_LOWER_DISTANCE",VISIBLE_LOWER_DISTANCE_,{5.0});
    private_nh_.param("VISIBLE_UPPER_DISTANCE",VISIBLE_UPPER_DISTANCE_,{0.3});
    private_nh_.param("ANGLE_OF_VIEW",ANGLE_OF_VIEW_,{86.0/180.0*M_PI});
	private_nh_.param("DISTANCE_NOISE",DISTANCE_NOISE_,{0.1});

	private_nh_.param("USE_OPS_MSG",USE_OPS_MSG_,{true});
	if(USE_OPS_MSG_){
    	ops_sub_ = nh_.subscribe("ops_in",1,&ObjectLocalizer::ops_callback,this);
		database_->load_init_objects();
		// robot_name_list_->print_elements();
		// database_->print_object_params();
		// database_->print_elements();
	}

	private_nh_.param("USE_OCPS_MSG",USE_OCPS_MSG_,{true});
	if(USE_OCPS_MSG_){
		ocps_sub_ = nh_.subscribe("ocps_in",1,&ObjectLocalizer::ocps_callback,this);
		// pose_subscribers_->print_elements();
	}

	private_nh_.param("PUBLISH_DATABASE",PUBLISH_DATABASE_,{true});

    init();
}

void multi_localizer::ObjectLocalizer::ops_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg) { filter_ops_msg(*msg,ops_); }

void multi_localizer::ObjectLocalizer::ocps_callback(const object_color_detector_msgs::ObjectColorPositionsConstPtr& msg) 
{ 
	ocps_ = *msg;
	for(const auto &ocp : ocps_.object_color_position){
		std::cout << "(Color,Dist,Angle): ("
		          << ocp.color << ","
				  << std::sqrt(ocp.x*ocp.x + ocp.z*ocp.z) << ","
				  << std::atan2(ocp.z,ocp.x) - 0.5*M_PI << ")" << std::endl;
	}
	std::cout <<  std::endl;
}

void multi_localizer::ObjectLocalizer::observation_update()
{
	// if(ops_.object_position.empty()) return;
    for(auto &p : particles_) p.weight_ = get_weight(p.pose_);
    normalize_particles_weight();
    calc_weight_params();
}

void multi_localizer::ObjectLocalizer::publish_tf()
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

bool multi_localizer::ObjectLocalizer::is_start()
{
	if(has_received_odom_) return true;
    return false;
}

double multi_localizer::ObjectLocalizer::get_weight(geometry_msgs::PoseStamped& pose)
{
	double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double yaw = tf2::getYaw(pose.pose.orientation);
    double weight = 0.0;
	auto weight_func = [](double mu,double sigma) -> double
    {
		return std::exp(-0.5*mu*mu/(sigma*sigma))/(std::sqrt(2.0*M_PI*sigma*sigma));
    };
	if(USE_OPS_MSG_){
		for(const auto & op : ops_.object_position){
			double distance = std::sqrt(op.x*op.x + op.z*op.z);
			double angle = std::atan2(op.z,op.x) - 0.5*M_PI;

			double sim;
			double tmp_yaw = tf2::getYaw(estimated_pose_.pose.orientation);
			double tmp_x = estimated_pose_.pose.position.x + distance*std::cos(tmp_yaw + angle);
			double tmp_y = estimated_pose_.pose.position.y + distance*std::sin(tmp_yaw + angle);
    		Object nearest_object = database_->get_nearest_object(op.Class,tmp_x,tmp_y,sim);
			std::cout << "Object(sim): " << op.Class << "(" << sim << ")" <<  std::endl;
			if(sim > 0.0){
		    	double hat_x = x + distance*std::cos(yaw + angle);
		    	double hat_y = y + distance*std::sin(yaw + angle);
		    	double error_x = hat_x - nearest_object.x;
		    	double error_y = hat_y - nearest_object.y;
		    	double sigma = DISTANCE_NOISE_*distance;
		    	weight += weight_func(error_x,sigma)*weight_func(error_y,sigma);
        	}
    	}
	}
	if(USE_OCPS_MSG_){
		for(const auto &ocp : ocps_.object_color_position){
			double distance = std::sqrt(ocp.x*ocp.x + ocp.z*ocp.z);
			double angle = std::atan2(ocp.z,ocp.x) - 0.5*M_PI;
        	double tmp_robot_x = x + distance*std::cos(yaw + angle);
        	double tmp_robot_y = y + distance*std::sin(yaw + angle);

        	geometry_msgs::PoseStamped robot_pose;
        	if(pose_subscribers_->get_pose(ocp.color,robot_pose)){
            	double error_x = tmp_robot_x - robot_pose.pose.position.x;
            	double error_y = tmp_robot_y - robot_pose.pose.position.y;
            	double sigma = DISTANCE_NOISE_*distance;
            	weight += weight_func(error_x,sigma)*weight_func(error_y,sigma);
        	}
		}
		// ocps_.object_color_position.clear();
	}
	return weight;
}

void multi_localizer::ObjectLocalizer::filter_ops_msg(object_detector_msgs::ObjectPositions input_ops,
                                                      object_detector_msgs::ObjectPositions& output_ops)
{
	output_ops.header = input_ops.header;
    output_ops.object_position.clear();

    auto is_visible_range = [this](object_detector_msgs::ObjectPosition op) -> bool
    {
        if(robot_name_list_->is_included(op.Class)) return false;
		if(op.Class == "fire_extinguisher") return false;
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

void multi_localizer::ObjectLocalizer::publish_objects_msg()
{
	double weight = get_max_particle_weight();
    // std::cout << "weight: " << weight << std::endl;
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
	ops_.object_position.clear();
}

void multi_localizer::ObjectLocalizer::process()
{
	ros::Rate rate(HZ_);
    while(ros::ok()){
        if(is_start()){
            database_->all_objects_are_not_observed();
            
            if(is_dense() || is_spread()){
                double tmp_x = estimated_pose_.pose.position.x;
                double tmp_y = estimated_pose_.pose.position.y;
                double tmp_yaw = tf2::getYaw(estimated_pose_.pose.orientation);
                set_particles(tmp_x,tmp_y,tmp_yaw,INIT_X_VAR_,INIT_Y_VAR_,INIT_YAW_VAR_);
            }

            motion_update();
			// if(USE_OPS_MSG_ && !ops_.object_position.empty()) observation_update();
   			observation_update();         
			if(is_update_){
                resample_particles();
                sort_by_particle_weight();
                // calc_estimated_pose();
                // calc_variance();
                is_update_ = false;
            }
            calc_estimated_pose();
            calc_variance();
            publish_particle_poses();
            publish_estimated_pose();
            publish_objects_msg();
			publish_tf();
        }
		if(PUBLISH_DATABASE_) database_->publish_markers();
        has_received_odom_ = false;
        previous_odom_ = current_odom_;
        ros::spinOnce();
        rate.sleep();
    }

}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"object_localizer");
	multi_localizer::ObjectLocalizer object_localizer;
	object_localizer.process();
	return 0;
}