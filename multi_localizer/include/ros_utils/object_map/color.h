#ifndef COLOR_H_
#define COLOR_H_

#include <iostream>

namespace multi_localizer
{
class Color
{
public:
    Color();
    Color(double _r,double _g,double _b);

    void print_color(); // for debug

    double r;
    double g;
    double b;

private:
};
} // namespace multi_localizer

#endif  // COLOR_H_
