#include "ros_utils/object_map/object.h"

using namespace object_localizer;

Object::Object() :
	is_observed(false), time(0.0), credibility(0.0), x(0.0), y(0.0) {}

Object::Object(double _time,double _credibility,double _x,double _y) :
	is_observed(false), time(_time), credibility(_credibility), x(_x), y(_y) {}

void Object::print_element()
{
	std::cout << "(Time,Cre,X,Y): ("
              << time << ","
			  << credibility << ","
			  << x << ","
			  << y << ")" << std::endl;
}

double Object::get_similarity(double _x,double _y)
{
	double error_x = x - _x;
    double error_y = y - _y;
	return get_weight(error_x)*get_weight(error_y);
}

double Object::get_weight(double error)
{
	double sigma = 1.0;
    return std::exp(-0.5*error*error/(sigma*sigma));
}