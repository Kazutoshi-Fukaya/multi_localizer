#include "ros_utils/robot_name_list/robot_name_list.h"

using namespace multi_localizer;

RobotNameList::RobotNameList()
{
	ROS_WARN("'robot_name_list' has not received 'private_nh'");
}

RobotNameList::RobotNameList(ros::NodeHandle private_nh)
{ 
	load_yaml(private_nh); 
}

void RobotNameList::load_yaml(ros::NodeHandle private_nh)
{
	std::string yaml_file_name;
	private_nh.param("YAML_FILE_NAME",yaml_file_name,{std::string("robot_name_list")});
	XmlRpc::XmlRpcValue robot_name_list;
    if(!private_nh.getParam(yaml_file_name.c_str(),robot_name_list)){
		ROS_ERROR("Cloud not load %s", yaml_file_name.c_str());
        return;
    }
	
	ROS_ASSERT(robot_name_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	for(int i = 0; i < robot_name_list.size(); i++){
		if(!robot_name_list[i]["name"].valid()){
			ROS_ERROR("%s is valid", yaml_file_name.c_str());
            return;
        }
        if(robot_name_list[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString){
			std::string name = static_cast<std::string>(robot_name_list[i]["name"]);
            this->emplace_back(name);
        }
    }
}

void RobotNameList::print_elements()
{
	for(auto it = this->begin(); it != this->end(); it++) std::cout << "name: " << *it << std::endl;
    std::cout << std::endl;
}

bool RobotNameList::is_included(std::string name)
{
	for(auto it = this->begin(); it != this->end(); it++){
		if(name ==*it) return true;
    }
    return false;
}
