#include "multi_pr_localizer/multi_pr_localizer.h"

using namespace multi_localizer;

MultiPRLocalizer::MultiPRLocalizer()
{
	// MCL Base
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

	// multi_pr_localizer params
	private_nh_.param("ERROR_TH",ERROR_TH_,{1.5});
}

MultiPRLocalizer::~MultiPRLocalizer() {}

void MultiPRLocalizer::pr_callback(const place_recognition_msgs::PoseStampedConstPtr& msg) { pr_pose_ = *msg; }

void MultiPRLocalizer::observation_update()
{
	if(!is_observation()) return;
	for(auto &p : particles_) p.weight_ = get_weight(p.pose_);
    normalize_particles_weight();
    calc_weight_params();
}

void MultiPRLocalizer::publish_tf()
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

bool MultiPRLocalizer::is_start()
{
	if(has_received_odom_) return true;
    return false;
}

bool MultiPRLocalizer::is_observation()
{
	double diff_x = estimated_pose_.pose.position.x - pr_pose_.x;
	double diff_y = estimated_pose_.pose.position.y - pr_pose_.y;
	double dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
	if(dist < ERROR_TH_) return true;
	return false;
}

double MultiPRLocalizer::get_weight(geometry_msgs::PoseStamped& pose)
{
	double weight = 0.0;
	double diff_x = pose.pose.position.x - pr_pose_.x;
	double diff_y = pose.pose.position.y - pr_pose_.y;
	double diff_theta = tf2::getYaw(pose.pose.orientation) - pr_pose_.theta;
	double score = pr_pose_.score;

	// position
	double dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
	weight += weight_func(dist,score);

	// orientation
	weight += weight_func(diff_theta,score);

	return weight;
}

double MultiPRLocalizer::weight_func(double mu,double sigma)
{
	return std::exp(-0.5*mu*mu/(sigma*sigma))/(std::sqrt(2.0*M_PI*sigma*sigma));
}

void MultiPRLocalizer::process()
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
            publish_tf();
		}
		has_received_odom_ = false;		
		previous_odom_ = current_odom_;
		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"multi_pr_localizer");
	MultiPRLocalizer multi_pr_localizer;
	multi_pr_localizer.process();
	return 0;
}