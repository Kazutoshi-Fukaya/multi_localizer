#ifndef OBJECT_H_
#define OBJECT_H_

#include <iostream>

class Object
{
public:
    Object() :
        is_observed(false), time(0.0), credibility(0.0), x(0.0), y(0.0) {}

    Object(double _time,double _credibility,double _x,double _y) :
        is_observed(false), time(_time), credibility(_credibility), x(_x), y(_y) {}

    double get_similarity(double _x,double _y)
    {
        // TO DO
        double error_x = x - _x;
        double error_y = y - _y;

        return get_weight(error_x)*get_weight(error_y);
    }

    // for debug
    void print_element()
    {
        std::cout << "(Time,Cre,X,Y): ("
                  << time << ","
                  << credibility << ","
                  << x << ","
                  << y << ")" << std::endl;
    }

    bool is_observed;
    double time;
    double credibility;
    double dom;
    double x;
    double y;

private:
    double get_weight(double error)
    {
        double sigma = 1.0;
        return std::exp(-0.5*error*error/(sigma*sigma));
    }
};

#endif  // OBJECT_H_
