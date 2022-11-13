#ifndef OBJECT_LOCALIZER_H_
#define OBJECT_LOCALIZER_H_

#include "mcl_base/mcl_base.h"
#include "robot_name_list/robot_name_list.h"
#include "database/database.h"
#include "pose_subscribers/pose_subscribers.h"

// c++
#include <random>
#include <fstream>
#include <sstream>

// Custom msg
#include "multi_robot_msgs/ObjectMap.h"
#include "multi_robot_msgs/ObjectsData.h"
#include "object_detector_msgs/ObjectPositions.h"
#include "object_color_detector_msgs/ObjectColorPositions.h"

namespace multi_localizer
{
class ObjectLocalizer : public MCLBase
{
public:
	ObjectLocalizer();
	void process();

private:
	void ops_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);
	void ocps_callback(const object_color_detector_msgs::ObjectColorPositionsConstPtr& msg);

	void observation_update();
	void publish_tf();
	bool is_start();
	bool is_observation();
	double get_weight(geometry_msgs::PoseStamped& pose);

	void filter_ops_msg(object_detector_msgs::ObjectPositions input_ops,
                        object_detector_msgs::ObjectPositions& output_ops);
	void publish_objects_msg();

	// subscriber
    ros::Subscriber ops_sub_;
	ros::Subscriber ocps_sub_;

	// time
    ros::Time start_time_;

	// utility
	RobotNameList* robot_name_list_;
	Database* database_;
	PoseSubscribers* pose_subscribers_;

	// buffer
	object_detector_msgs::ObjectPositions ops_;
	object_color_detector_msgs::ObjectColorPositions ocps_;

	// parameters
	bool USE_OPS_MSG_;
	bool USE_OCPS_MSG_;
	bool PUBLISH_DATABASE_;
	double PROBABILITY_TH_;
    double VISIBLE_LOWER_DISTANCE_;
    double VISIBLE_UPPER_DISTANCE_;
    double ANGLE_OF_VIEW_;
	double DISTANCE_NOISE_;
};
}

#endif	// OBJECT_LOCALIZER_H_