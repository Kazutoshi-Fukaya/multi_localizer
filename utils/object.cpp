#include "utils/object_map/object.h"

using namespace multi_localizer;

Object::Object() :
	has_observed(false), time(0.0), credibility(0.0), x(0.0), y(0.0) {}

Object::Object(double _time,double _credibility,double _x,double _y) :
    has_observed(false), time(_time), credibility(_credibility), x(_x), y(_y) {}

double Object::get_similarity(double _x,double _y)
{
    double sigma = 1.0;
    double error_x = x - _x;
    double error_y = y - _y;
    return weight_func(error_x,sigma)*weight_func(error_y,sigma);
}

// for debug
void Object::print_element()
{
    std::cout << "(Time,Cre,X,Y): ("
              << time << ","
              << credibility << ","
              << x << ","
              << y << ")" << std::endl;
}

double Object::weight_func(double error,double sigma) {  return std::exp(-0.5*error*error/(sigma*sigma)); }