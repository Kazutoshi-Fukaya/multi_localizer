#ifndef OBJECTS_H_
#define OBJECTS_H_

#include <algorithm>
#include <iterator>
#include <vector>

#include "ros_utils/object_map/object.h"

namespace object_localizer
{
class Objects : public std::vector<Object>
{
public:
    Objects();

    void add_object(Object object);
    void print_elements();  // for debug

    Object get_nearest_object(double x,double y,double& sim);
    
private:
};
} // namespace object_localizer

#endif  // OBJECTS_H_