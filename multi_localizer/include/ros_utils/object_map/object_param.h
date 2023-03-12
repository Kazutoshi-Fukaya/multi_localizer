#ifndef OBJECT_PARAM_H_
#define OBJECT_PARAM_H_

#include "ros_utils/object_map/color.h"

namespace multi_localizer
{
class ObjectParam
{
public:
    ObjectParam(std::string _name,bool _is_static,Color _color);
    ObjectParam(std::string _name,std::string _condition,Color _color);

    void print_element();   // for debug
    void print_param();     // for debug

    std::string print_condition();
    bool is_included(std::string _name);

    // params
    std::string name;
    bool is_static;
    Color color;

private:
    bool str_to_bool(std::string condition);
};
} // namespace multi_localizer

#endif  // OBJECT_PARAM_H_
