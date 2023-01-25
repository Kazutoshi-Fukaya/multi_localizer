#ifndef SINGLE_MCL_H_
#define SINGLE_MCL_H_

// mcl base
#include "mcl_base/mcl_base.h"

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom msg
#include "multi_localizer_msgs/ObjectsData.h"
#include "multi_localizer_msgs/RobotPoseStamped.h"
#include "object_detector_msgs/ObjectPositions.h"

namespace multi_localizer
{
class MCL : public MCLBase
{
public:
    MCL();
    ~MCL();
    void process();

private:
    void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void lsr_callback(const sensor_msgs::LaserScanConstPtr& msg);
    void od_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);

    // mcl base
    void observation_update();
    void publish_tf();
    bool is_start();
    double get_weight(geometry_msgs::PoseStamped& pose);

    // mcl
    void publish_robot_pose();
    double get_dist_to_wall(double x,double y,double yaw);

    // subscriber
    ros::Subscriber map_sub_;
    ros::Subscriber lsr_sub_;
    ros::Subscriber od_sub_;

    // publisher
    ros::Publisher robot_pose_pub_;
    ros::Publisher data_pub_;

    // buffer (for mcl)
    ros::Time start_time_;
    nav_msgs::OccupancyGrid static_map_;
    sensor_msgs::LaserScan lsr_;
    bool has_received_map_;

    // mcl parameters
    bool PUBLISH_OBJECTS_DATA_;
    std::string ROBOT_NAME_;
    int RANGE_STEP_;
    double MAX_RANGE_;
    double HIT_COV_;
    double LAMBDA_SHORT_;
    double Z_HIT_;
    double Z_SHORT_;
    double Z_MAX_;
    double Z_RAND_;
};
} // namespace multi_localizer

#endif  // SINGLE_MCL_H_