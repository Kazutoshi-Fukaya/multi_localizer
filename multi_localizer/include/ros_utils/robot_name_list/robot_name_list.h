#ifndef ROBOT_NAME_H_
#define ROBOT_NAME_H_

#include <ros/ros.h>

namespace multi_localizer
{
class RobotNameList : public std::vector<std::string>
{
public:
    RobotNameList();
    RobotNameList(ros::NodeHandle _private_nh);

    void print_elements();  // for debug
    bool is_included(std::string name);

private:
    void load_yaml(ros::NodeHandle private_nh);
};
} // namespace multi_localizer

#endif  // ROBOT_NAME_H_