#ifndef OBJECT_PARAM_H_
#define OBJECT_PARAM_H_

#include "utils/object_param/color.h"

namespace multi_localizer
{
class ObjectParam
{
public:
    ObjectParam(std::string _name,bool _is_static,Color _color) :
        name(_name), is_static(_is_static), color(_color) {}

    ObjectParam(std::string _name,std::string _condition,Color _color) :
        name(_name), is_static(str_to_bool(_condition)), color(_color) {}

    bool is_included(std::string _name)
    {
        if(_name == name) return true;
        return false;
    }

    // for debug
    void print_element()
    {
        print_param();
        color.print_color();
        std::cout << std::endl;
    }

    void print_param()
    {
        std::cout << "Name: " << name << std::endl;
        std::cout << "Condition: " << print_condition() << std::endl;
    }

    std::string print_condition()
    {
        if(is_static) return std::string("Static");
        return std::string("Semi - Dynamic");
    }

    std::string name;
    bool is_static;
    Color color;

private:
    bool str_to_bool(std::string condition)
    {
        if(condition == std::string("Static")) return true;
        return false;
    }

};
} // namespace multi_localizer

#endif  // OBJECT_PARAM_H_
