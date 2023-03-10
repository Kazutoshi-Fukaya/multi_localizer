#ifndef ROBOT_ELEMENT_H_
#define ROBOT_ELEMENT_H_

#include <iostream>

namespace relative_recognition_localizer
{
class RobotElement
{
public:
    RobotElement();
    RobotElement(std::string _robot_name,std::string _color);

    std::string robot_name;
    std::string color;
private:
};
} // namespace relative_recognition_localizer

#endif  // ROBOT_ELEMENT_H_