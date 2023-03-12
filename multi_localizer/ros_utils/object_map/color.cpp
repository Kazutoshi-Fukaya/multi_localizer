#include "ros_utils/object_map/color.h"

using namespace multi_localizer;

Color::Color() :
	r(0.0), g(0.0), b(0.0) {}

Color::Color(double _r,double _g,double _b) :
	r(_r),g(_g), b(_b) {}

void Color::print_color()
{
	std::cout << "Color(R,G,B): ("
              << r << "," << g << "," << b << ")" << std::endl;
}