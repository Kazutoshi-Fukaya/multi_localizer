#ifndef ROBOT_NAME_H_
#define ROBOT_NAME_H_

#include <ros/ros.h>

namespace multi_localizer
{
class RobotNameList : public std::vector<std::string>
{
public:
    RobotNameList() :
        private_nh_("~") { load_yaml(); }

    RobotNameList(ros::NodeHandle _private_nh) :
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
        private_nh_.param("YAML_FILE_NAME",yaml_file_name,{std::string("robot_name_list")});
        XmlRpc::XmlRpcValue robot_name_list;
        if(!private_nh_.getParam(yaml_file_name.c_str(),robot_name_list)){
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

    ros::NodeHandle private_nh_;
};
} // namespace multi_localizer

#endif  // ROBOT_NAME_H_