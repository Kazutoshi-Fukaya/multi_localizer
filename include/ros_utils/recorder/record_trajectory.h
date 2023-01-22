#ifndef RECORD_TRAJECTORY_H_
#define RECORD_TRAJECTORY_H_

#include "ros_utils/recorder/record_pose.h"

namespace multi_localizer
{
class RecordTrajectory
{
public:
	double time;
    RecordPose est_pose;
    RecordPose ref_pose;

private:
};
} // namespace multi_localizer

#endif	// RECORD_TRAJECTORY_H_