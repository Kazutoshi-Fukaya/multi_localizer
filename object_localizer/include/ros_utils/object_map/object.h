#ifndef OBJECT_H_
#define OBJECT_H_

#include <iostream>
#include <cmath>

namespace object_localizer
{
class Object
{
public:
    Object();
    Object(double _time,double _credibility,double _x,double _y);

    void print_element();   // for debug
    double get_similarity(double _x,double _y);

    // params
    bool is_observed;
    double time;
    double credibility;
    double dom;
    double x;
    double y;

private:
    double get_weight(double error);
};
} // namespace object_localizer

#endif  // OBJECT_H_