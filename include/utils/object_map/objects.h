#ifndef OBJECTS_H_
#define OBJECTS_H_

#include <vector>
#include <iterator>
#include <algorithm>

#include "utils/object_map/object.h"

namespace multi_localizer
{
class Objects : public std::vector<Object>
{
public:
    Objects();

    void add_init_object(Object object);
    Object get_highly_similar_object(double x,double y,double& sim);

    // for debug
    void print_elements();

private:
};
} // namespace multi_localizer

#endif  // OBJECTS_H_