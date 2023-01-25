#ifndef DYNAMIC_OBJECTS_H_
#define DYNAMIC_OBJECTS_H_

#include <ros/ros.h>

class DynamicObjects : public std::vector<std::string>
{
public:
    DynamicObjects() :
        private_nh_("~") { load_yaml(); }

    DynamicObjects(ros::NodeHandle _private_nh) :
        private_nh_(_private_nh) { load_yaml(); }

    bool is_included(std::string name)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(name ==*it) return true;
        }
        return false;
    }

    // for debug
    void print_elements()
    {
        for(auto it = this->begin(); it != this->end(); it++) std::cout << "name: " << *it << std::endl;
        std::cout << std::endl;
    }

private:
    void load_yaml()
    {
        std::string yaml_file_name;
        private_nh_.param("YAML_FILE_NAME",yaml_file_name,{std::string("dynamic_objects")});
        XmlRpc::XmlRpcValue dynamic_objects;
        if(!private_nh_.getParam(yaml_file_name.c_str(),dynamic_objects)){
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

    ros::NodeHandle private_nh_;
};

#endif  // DYNAMIC_OBJECTS_H_
