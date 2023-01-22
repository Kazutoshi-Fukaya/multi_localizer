#ifndef OBJECT_H_
#define OBJECT_H_

#include <iostream>
#include <cmath>

namespace multi_localizer
{
class Object
{
public:
    Object();
    Object(double _time,double _credibility,double _x,double _y);

    double get_similarity(double _x,double _y);

    // for debug
    void print_element();

    bool has_observed;  // Observed or not
    double time;        // Time 
    double credibility; // Credibility 
    double dom;         // difficulty of moving estimated by 'dom_estimator'
    double x;           // x-coordinate estimated by 'dom_estimator'
    double y;           // y-coordinate estimated by 'dom_estimator'

private:
    double weight_func(double error,double sigma);
    
};
} // namespace multi_localizer

#endif  // OBJECT_H_
