#include "ros_utils/object_params/object_param.h"

using namespace object_localizer;

ObjectParam::ObjectParam() :
	name(std::string("")), is_static(false), color(Color()) {}

ObjectParam::ObjectParam(std::string _name,std::string _condition,Color _color) :
	name(_name), is_static(convert_from_str_to_bool(_condition)), color(_color) {}

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

bool ObjectParam::convert_from_str_to_bool(std::string condition)
{
	if(condition == std::string("Static")) return true;
    return false;
}

bool ObjectParam::is_included(std::string _name)
{
	if(name == _name) return true;
    return false;
}