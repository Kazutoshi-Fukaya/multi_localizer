#ifndef RECORD_POSE_H_
#define RECORD_POSE_H_

namespace multi_localizer
{
class RecordPose
{
public:
    RecordPose() :
        x(0.0), y(0.0), yaw(0.0) {}

    RecordPose(double _x,double _y,double _yaw) :
        x(_x), y(_y), yaw(_yaw) {}

    double x;
    double y;
    double yaw;

private:
};
} // namespace multi_localizer

#endif  // RECORD_POSE_H_
