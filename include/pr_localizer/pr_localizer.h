#ifndef PR_LOCALIZER_H_
#define PR_LOCALIZER_H_

#include <ros/ros.h>

#include "mcl_base/mcl_base.h"

#include "place_recognition_msgs/PoseStamped.h"

namespace multi_localizer
{
class PRLocalizer : public MCLBase
{
public:
	PRLocalizer();
	~PRLocalizer();
	void process();

private:
	// callback (for pr_localizer)
	void pr_callback(const place_recognition_msgs::PoseStampedConstPtr& msg);

	// mcl base
	void observation_update();
    void publish_tf();
    bool is_start();
    bool is_observation();
    double get_weight(geometry_msgs::PoseStamped& pose);

	// for pr localizer
	double weight_func(double mu,double sigma);

	// subscriber
	ros::Subscriber pr_pose_sub_;

	// buffer
	place_recognition_msgs::PoseStamped pr_pose_;

	// multi_pr_localizer params
	double ERROR_TH_;
	double SCORE_TH_;
};
} // namespace multi_localizer

#endif	// PR_LOCALIZER_H_