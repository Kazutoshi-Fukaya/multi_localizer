#ifndef SINGLE_MCL_H_
#define SINGLE_MCL_H_

#include "mcl_base/mcl_base.h"

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

namespace multi_localizer
{
class SingleMCL : public MCLBase
{
public:
	SingleMCL();
	void process();

private:
	void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void lsr_callback(const sensor_msgs::LaserScanConstPtr& msg);

	void observation_update();
	void publish_tf();
	bool is_start();
	double get_weight(geometry_msgs::PoseStamped& pose);

	double get_dist_to_wall(double x,double y,double yaw);

	// subscriber
	ros::Subscriber map_sub_;
    ros::Subscriber lsr_sub_;

	// buffer
	nav_msgs::OccupancyGrid static_map_;
    sensor_msgs::LaserScan lsr_;
    bool has_received_map_;

	// parameters
	int RANGE_STEP_;
	double MAX_RANGE_;
	double HIT_COV_;
    double LAMBDA_SHORT_;
    double Z_HIT_;
    double Z_SHORT_;
    double Z_MAX_;
    double Z_RAND_;
};
}

#endif	// SINGLE_MCL_H_