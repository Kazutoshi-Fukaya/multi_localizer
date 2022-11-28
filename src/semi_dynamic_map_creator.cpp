#include "dynamic_mcl/semi_dynamic_map_creator.h"

using namespace multi_localizer;

SemiDynamicMapCreator::SemiDynamicMapCreator() :
	private_nh_("~")
{
	private_nh_.param("LOCAL_MAP_FRAME_ID",LOCAL_MAP_FRAME_ID_,{std::string("base_link")});
	private_nh_.param("HAS_REMOVED_POLE",HAS_REMOVED_POLE_,{true});
	private_nh_.param("WIDTH",WIDTH_,{5});
    private_nh_.param("HEIGHT",HEIGHT_,{5});
	private_nh_.param("RANGE_STEP",RANGE_STEP_,{5});
    private_nh_.param("RESOLUTION",RESOLUTION_,{0.05});
	private_nh_.param("ROBOT_RADIUS",ROBOT_RADIUS_,{0.2});
	private_nh_.param("POLE_MARGIN",POLE_MARGIN_,{0.2});

	lsr_sub_ = nh_.subscribe("lsr_in",1,&SemiDynamicMapCreator::lsr_callback,this);
	pose_sub_ = nh_.subscribe("pose_in",1,&SemiDynamicMapCreator::pose_callback,this);

	map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_out",1);

	generate_map();
}

void SemiDynamicMapCreator::lsr_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
	lsr_ = *msg;
	init_map();
	update_map();
	map_pub_.publish(local_map_);
}

void SemiDynamicMapCreator::pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{

}

void SemiDynamicMapCreator::generate_map()
{
	local_map_.header.frame_id = LOCAL_MAP_FRAME_ID_;
    local_map_.info.resolution = RESOLUTION_;
    local_map_.info.width = (int)(WIDTH_/RESOLUTION_);
    local_map_.info.height = (int)(HEIGHT_/RESOLUTION_);
    local_map_.info.origin.position.x = -WIDTH_/2;
    local_map_.info.origin.position.y = -HEIGHT_/2;
    local_map_.data.resize(local_map_.info.width*local_map_.info.height);
	init_map();
}

void SemiDynamicMapCreator::init_map()
{
    local_map_.data.clear();
    for(int i = 0; i < (int)(local_map_.info.width*local_map_.info.height); i++){
        local_map_.data.emplace_back(-1);
    }
}

void SemiDynamicMapCreator::update_map()
{
    obstacle_poses_.poses.clear();
    for(size_t i = 0; i < lsr_.ranges.size(); i += RANGE_STEP_){
        double angle = i*lsr_.angle_increment + lsr_.angle_min;
        input_to_map(angle,lsr_.ranges[i]);
    }
}

void SemiDynamicMapCreator::input_to_map(double angle,double lsr_range)
{
    if(!HAS_REMOVED_POLE_){
        if(lsr_range <= ROBOT_RADIUS_ || is_pole(angle)) lsr_range = WIDTH_;
    }

    for(double d = 0.0; d < WIDTH_; d += RESOLUTION_){
        double search_x = d*std::cos(angle);
        double search_y = d*std::sin(angle);

        if(!is_in_map(search_x,search_y)) return;

        if(d >= lsr_range){
            local_map_.data[coordinate_to_map_index(search_x,search_y)] = 100;
            geometry_msgs::Pose obstacle_pose;
            obstacle_pose.position.x = search_x;
            obstacle_pose.position.y = search_y;
            obstacle_poses_.poses.push_back(obstacle_pose);
            return;
        }
        else{
            local_map_.data[coordinate_to_map_index(search_x,search_y)] = 0;
        }
    }
}

bool SemiDynamicMapCreator::is_pole(double angle)
{
    // back left
    if(-0.75*M_PI <= angle && angle <= -0.75*M_PI + POLE_MARGIN_) return true;
    // front left
    else if(-0.25*M_PI - POLE_MARGIN_ <= angle && angle <= -0.25*M_PI + POLE_MARGIN_) return true;
    // front right
    else if(0.25*M_PI - POLE_MARGIN_ <= angle && angle <= 0.25*M_PI + POLE_MARGIN_) return true;
    // back right
    else if(0.75*M_PI - POLE_MARGIN_ <= angle && angle <= 0.75*M_PI) return true;

    // is not pole
    return false;
}

bool SemiDynamicMapCreator::is_in_map(double x,double y)
{
    double x_min = local_map_.info.origin.position.x;
    double x_max = x_min + local_map_.info.width*local_map_.info.resolution;
    double y_min = local_map_.info.origin.position.y;
    double y_max = y_min + local_map_.info.height*local_map_.info.resolution;

    if(x_min < x && x < x_max && y_min < y && y < y_max) return true;
    return false;
}

int SemiDynamicMapCreator::coordinate_to_map_index(double x,double y)
{
    int index_x = std::floor((x - local_map_.info.origin.position.x)/local_map_.info.resolution);
    int index_y = std::floor((y - local_map_.info.origin.position.y)/local_map_.info.resolution);

    return index_x + index_y*local_map_.info.width;
}

void SemiDynamicMapCreator::process() { ros::spin(); }

int main(int argc,char** argv)
{
	ros::init(argc,argv,"semi_dynamic_map_creator");
	SemiDynamicMapCreator semi_dynamic_map_creator;
	semi_dynamic_map_creator.process();
	return 0;
}