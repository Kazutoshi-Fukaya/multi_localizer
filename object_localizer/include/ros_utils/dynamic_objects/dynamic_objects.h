#ifndef DYNAMIC_OBJECTS_H_
#define DYNAMIC_OBJECTS_H_

#include <ros/ros.h>

namespace object_localizer
{
class DynamicObjects : public std::vector<std::string>
{
public:
    DynamicObjects();
    DynamicObjects(ros::NodeHandle private_nh);

    bool is_included(std::string name);
    void print_elements();  // for debug

private:
    void load_yaml(ros::NodeHandle private_nh);
};
} // namespace object_localizer

#endif  // DYNAMIC_OBJECTS_H_