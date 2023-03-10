#include "ros_utils/dynamic_objects/dynamic_objects.h"

using namespace object_localizer;

DynamicObjects::DynamicObjects() 
{
	ROS_WARN("'dynamic objects' has not received 'private_nh'");
}

DynamicObjects::DynamicObjects(ros::NodeHandle private_nh)
{ 
	load_yaml(private_nh); 
}

void DynamicObjects::load_yaml(ros::NodeHandle private_nh)
{
	std::string yaml_file_name;
	private_nh.param("YAML_FILE_NAME",yaml_file_name,{std::string("dynamic_objects")});
    XmlRpc::XmlRpcValue dynamic_objects;
    if(!private_nh.getParam(yaml_file_name.c_str(),dynamic_objects)){
		ROS_ERROR("Cloud not load %s", yaml_file_name.c_str());
        return;
    }

    ROS_ASSERT(dynamic_objects.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < dynamic_objects.size(); i++){
		if(!dynamic_objects[i]["name"].valid()){
			ROS_ERROR("%s is valid", yaml_file_name.c_str());
            return;
        }
        if(dynamic_objects[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString){
			std::string name = static_cast<std::string>(dynamic_objects[i]["name"]);
            this->emplace_back(name);
        }
    }
}

void DynamicObjects::print_elements()
{
	std::cout << "=== DYNAMIC OBJECTS ===" << std::endl;
	for(auto it = this->begin(); it != this->end(); it++){
		std::cout << "name: " << *it << std::endl;
        std::cout << std::endl;
    }
}

bool DynamicObjects::is_included(std::string name)
{
	for(auto it = this->begin(); it != this->end(); it++){
		if(name ==*it) return true;
    }
    return false;
}