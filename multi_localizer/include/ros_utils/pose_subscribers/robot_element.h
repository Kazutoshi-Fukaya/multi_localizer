#ifndef ROBOT_ELEMENT_H_
#define ROBOT_ELEMENT_H_

#include <iostream>

namespace multi_localizer
{
class RobotElement
{
public:
    RobotElement();        
    RobotElement(std::string _robot_name,std::string _color);

    // params
    std::string robot_name;
    std::string color;
private:
};
} // namespace multi_localizer

#endif  // ROBOT_ELEMENT_H_
