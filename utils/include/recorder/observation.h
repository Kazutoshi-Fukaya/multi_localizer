#ifndef OBSERVATION_H_
#define OBSERVATION_H_

#include <iostream>

namespace recorder
{
class Observation
{
public:
    Observation() :
        time(0.0), name(std::string("")) {}

    Observation(double _time,std::string _name) :
        time(_time), name(_name) {}

    double time;
    std::string name;
private:
};
} // namespace recorder

#endif  // OBSERVATION_H_
