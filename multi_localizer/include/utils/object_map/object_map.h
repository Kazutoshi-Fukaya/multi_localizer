#ifndef OBJECT_MAP_H_
#define OBJECT_MAP_H_

// ros
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

// c++
#include <map>
#include <sstream>
#include <fstream>

// utils
#include "utils/object_param/object_param.h"
#include "utils/object_map/objects.h"

namespace multi_localizer
{
class ObjectMap : public std::map<ObjectParam*,Objects*>
{
public:
    ObjectMap();

    void add_init_object(std::string name,Object object);
    void clear_objects_data();
    void all_objects_are_not_observed();
    Object get_highly_similar_object(std::string name,double x,double y,double& sim);

    // for debug
    void print_object_params();
    void print_elements();

private:
};
} // namespace multi_localizer

#endif  // OBJECT_MAP_H_
