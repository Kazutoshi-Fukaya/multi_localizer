#ifndef SINGLE_MCL_H_
#define SINGLE_MCL_H_

#include "mcl_base/mcl_base.h"
// #include "ros_utils/recorder/recorder.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom msg
#include "multi_robot_msgs/ObjectsData.h"
#include "object_detector_msgs/ObjectPositions.h"

namespace multi_localizer
{
class SingleMCL : public MCLBase
{
public:
    SingleMCL();
    ~SingleMCL();
    void process();

private:
    void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void lsr_callback(const sensor_msgs::LaserScanConstPtr& msg);
    // void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    // void obj_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);

    void observation_update();
    void publish_tf();
    // void record_pose();
    bool is_start();
    double get_weight(geometry_msgs::PoseStamped& pose);

    double get_dist_to_wall(double x,double y,double yaw);

    // subscriber
    ros::Subscriber map_sub_;
    ros::Subscriber lsr_sub_;

    // subscriber (for recording)
    // ros::Subscriber pose_sub_;
    // ros::Subscriber obj_sub_;

    // publish (for publishing objects data)
    // ros::Publisher obj_pub_;

    // recorder
    // Recorder* recorder_;

    // buffer
    ros::Time start_time_;
    nav_msgs::OccupancyGrid static_map_;
    sensor_msgs::LaserScan lsr_;
    geometry_msgs::PoseStamped ref_pose_;
    bool has_received_map_;

    // parameters
    // bool IS_RECORD_;
    // bool PUBLISH_OBJ_DATA_;
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