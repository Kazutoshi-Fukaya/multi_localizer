#ifndef RECORDER_H_
#define RECORDER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/utils.h>

#include <fstream>
#include <sstream>

#include "object_detector_msgs/ObjectPositions.h"

#include "ros_utils/recorder/record_trajectory.h"
#include "ros_utils/recorder/record_observation.h"

namespace multi_localizer
{
class Recorder
{
public:
    Recorder();
    void process();

private:
    void ref_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void est_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void obs_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);

    void add_trajectory();
    void save_csv();
    std::string get_date();
    double get_time();

    // node handler
    ros::NodeHandle private_nh_;

    // buffer
    ros::Time start_time_;
    RecordPose est_pose_;
    RecordPose ref_pose_;
    std::vector<RecordTrajectory> trajectories_;
    std::vector<RecordObservation> observations_;

    // params
    int HZ_;
};
} // namespace multi_localizer

#endif  // RECORDER_H_