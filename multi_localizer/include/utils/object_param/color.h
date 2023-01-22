#ifndef COLOR_H_
#define COLOR_H_

#include <iostream>

namespace multi_localizer
{
class Color
{
public:
    Color() :
        r(0.0), g(0.0), b(0.0) {}

    Color(double _r,double _g,double _b) :
        r(_r),g(_g), b(_b) {}

    // for debug
    void print_color()
    {
        std::cout << "Color(R,G,B): ("
                  << r << "," << g << "," << b << ")" << std::endl;
    }

    double r;
    double g;
    double b;

private:
};
} // namespace multi_localizer

#endif  // COLOR_H_
