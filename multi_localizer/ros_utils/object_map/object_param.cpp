#include "ros_utils/object_map/object_param.h"

using namespace multi_localizer;

ObjectParam::ObjectParam(std::string _name,bool _is_static,Color _color) :
	name(_name), is_static(_is_static), color(_color) {}

ObjectParam::ObjectParam(std::string _name,std::string _condition,Color _color) :
	name(_name), is_static(str_to_bool(_condition)), color(_color) {}

void ObjectParam::print_element()
{
	print_param();
	color.print_color();
	std::cout << std::endl;
}

void ObjectParam::print_param()
{
    std::cout << "Name: " << name << std::endl;
    std::cout << "Condition: " << print_condition() << std::endl;
}

std::string ObjectParam::print_condition()
{
	if(is_static) return std::string("Static");
    return std::string("Semi-Dynamic");
}

bool ObjectParam::is_included(std::string _name)
{
	if(_name == name) return true;
    return false;
}

bool ObjectParam::str_to_bool(std::string condition)
{
	if(condition == std::string("Static")) return true;
    return false;
}