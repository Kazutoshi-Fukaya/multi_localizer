#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "recorder/pose.h"

namespace recorder
{
class Trajectory
{
public:
    double time;
    Pose est_pose;
    Pose ref_pose;

private:
};
} // namespace recorder

#endif  // TRAJECTORY_H_
