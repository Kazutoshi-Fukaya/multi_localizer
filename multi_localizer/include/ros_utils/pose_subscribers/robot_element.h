#ifndef ROBOT_ELEMENT_H_
#define ROBOT_ELEMENT_H_

#include <iostream>

namespace multi_localizer
{
class RobotElement
{
public:
    RobotElement() :
        robot_name(std::string("")), color(std::string("")) {}
        
    RobotElement(std::string _robot_name,std::string _color) :
        robot_name(_robot_name), color(_color) {}

    std::string robot_name;
    std::string color;
private:
};
} // namespace multi_localizer

#endif  // ROBOT_ELEMENT_H_
