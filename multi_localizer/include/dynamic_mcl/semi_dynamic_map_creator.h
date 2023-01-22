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
#include "dynamic_mcl/semi_dynamic_objects.h"
#include "object_detector_msgs/ObjectPositions.h"

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
    void obj_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void generate_map();
    void init_map();
    void update_map();
    void input_to_map(double angle,double lsr_range);
    void filter_ops_msg(object_detector_msgs::ObjectPositions input_ops,
                        object_detector_msgs::ObjectPositions& output_ops);
    void update_latest_map(visualization_msgs::MarkerArray obj_poses);

    // TO DO
    void add_map_pose(geometry_msgs::PoseArray obs_poses);
    geometry_msgs::PoseArray get_obs(geometry_msgs::Pose pose);

	geometry_msgs::Pose get_pose_msg(double x,double y);
    bool is_pole(double angle);
    bool is_in_map(double x,double y);
    int coordinate_to_map_index(double x,double y);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber lsr_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber obj_sub_;
    ros::Subscriber map_sub_;

    // publisher
    ros::Publisher map_pub_;
    ros::Publisher latest_map_pub_;
    ros::Publisher obs_pub_;
    ros::Publisher markers_pub_;

    // buffer
    nav_msgs::OccupancyGrid local_map_;
    nav_msgs::OccupancyGrid latest_map_;
	geometry_msgs::PoseArray obs_poses_;
    geometry_msgs::PoseStamped pose_;
    sensor_msgs::LaserScan lsr_;
    bool has_received_pose_;
    bool has_received_map_;

    // semi-dynamic-objects
    SemiDynamicObjects* semi_dynamic_objects_;

    // params
    std::string LOCAL_MAP_FRAME_ID_;
	std::string MAP_FRAME_ID_;
    bool HAS_REMOVED_POLE_;
    int WIDTH_;
    int HEIGHT_;
    int RANGE_STEP_;
    double RESOLUTION_;
    double ROBOT_RADIUS_;
    double POLE_MARGIN_;
    double PROBABILITY_TH_;
    double VISIBLE_LOWER_DISTANCE_;
    double VISIBLE_UPPER_DISTANCE_;
    const double ANGLE_OF_VIEW_ = 86.0/180.0*M_PI;	// realsense D455
};
}

#endif  // SEMI_DYNAMIC_MAP_CREATOR_H_
