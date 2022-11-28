#ifndef SEMI_DYNAMIC_MAP_CREATOR_H_
#define SEMI_DYNAMIC_MAP_CREATOR_H_

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>

// Custom msg
// #include "object_detector_msgs/ObjectPositions.h"

namespace multi_localizer
{
class SemiDynamicMapCreator
{
public:
	SemiDynamicMapCreator();
	void process();

private:
	void lsr_callback(const sensor_msgs::LaserScanConstPtr& msg);
	void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);

	void generate_map();
	void init_map();
	void update_map();
	void input_to_map(double angle,double lsr_range);
	bool is_pole(double angle);
	bool is_in_map(double x,double y);
	int coordinate_to_map_index(double x,double y);

	// node handler
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// subscriber
	ros::Subscriber lsr_sub_;
	ros::Subscriber pose_sub_;

	// publisher
	ros::Publisher map_pub_;


	// buffer
	nav_msgs::OccupancyGrid local_map_;
	geometry_msgs::PoseArray obstacle_poses_;
	sensor_msgs::LaserScan lsr_;

	// params
	std::string LOCAL_MAP_FRAME_ID_;
	bool HAS_REMOVED_POLE_;
	int WIDTH_;
	int HEIGHT_;
	int RANGE_STEP_;
	double RESOLUTION_;
	double ROBOT_RADIUS_;
	double POLE_MARGIN_;

};
}

#endif  // SEMI_DYNAMIC_MAP_CREATOR_H_