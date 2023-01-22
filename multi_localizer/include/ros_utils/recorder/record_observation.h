#ifndef RECORD_OBSERVATION_H_
#define RECORD_OBSERVATION_H_

#include <iostream>

namespace multi_localizer
{
class RecordObservation
{
public:
    RecordObservation() :
        time(0.0), name(std::string("")) {}

    RecordObservation(double _time,std::string _name) :
        time(_time), name(_name) {}

    double time;
    std::string name;
private:
};
} // namespace multi_localizer

#endif  // RECORD_OBSERVATION_H_
