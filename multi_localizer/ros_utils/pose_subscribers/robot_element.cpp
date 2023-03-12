#include "ros_utils/pose_subscribers/robot_element.h"

using namespace multi_localizer;

RobotElement::RobotElement() :
	robot_name(std::string("")), color(std::string("")) {}
        
RobotElement::RobotElement(std::string _robot_name,std::string _color) :
	robot_name(_robot_name), color(_color) {}