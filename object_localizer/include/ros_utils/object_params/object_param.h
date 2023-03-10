#ifndef OBJECT_PARAM_H_
#define OBJECT_PARAM_H_

#include "ros_utils/object_params/color.h"

namespace object_localizer
{
class ObjectParam
{
public:
    ObjectParam();
    ObjectParam(std::string _name,std::string _condition,Color _color);

    bool convert_from_str_to_bool(std::string condition);
    bool is_included(std::string _name);

    // for debug
    void print_element();
    void print_param();
    std::string print_condition();

    std::string name;
    bool is_static;
    Color color;

private:
};
} // namespace object_localizer

#endif  // OBJECT_PARAM_H_