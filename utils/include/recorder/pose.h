#ifndef POSE_H_
#define POSE_H_

namespace recorder
{
class Pose
{
public:
    Pose() :
        x(0.0), y(0.0), yaw(0.0) {}

    Pose(double _x,double _y,double _yaw) :
        x(_x), y(_y), yaw(_yaw) {}

    double x;
    double y;
    double yaw;

private:
};
} // namespace recorder

#endif  // POSE_H_
