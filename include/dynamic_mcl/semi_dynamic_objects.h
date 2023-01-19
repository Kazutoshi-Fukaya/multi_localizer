#ifndef SEMI_DYNAMIC_OBJECTS_H_
#define SEMI_DYNAMIC_OBJECTS_H_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

namespace multi_localizer
{
class SemiDynamicObject
{
public:
    SemiDynamicObject() :
        name_(std::string("")), r_(0.0), g_(0.0), b_(0.0) {}
    SemiDynamicObject(std::string _name,double _r,double _g,double _b) :
        name_(_name), r_(_r), g_(_g), b_(_b) {}

    std::string name_;
    double r_;
    double g_;
    double b_;

private:
};

class SemiDynamicObjects : public std::vector<SemiDynamicObject>
{
public:
    SemiDynamicObjects() :
        private_nh_("~") { load_yaml(); }
    SemiDynamicObjects(ros::NodeHandle _private_nh) :
        private_nh_(_private_nh) { load_yaml(); }

    std_msgs::ColorRGBA get_color_msg(std::string name)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->name_ == name) return set_color_msg(it->r_,it->g_,it->b_);
        }
        return set_color_msg(1.0,1.0,1.0);
    }

    bool is_included(std::string name)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->name_ == name) return true;
        }
        return false;
    }

private:
    void load_yaml()
    {
        std::string yaml_file_name;
        private_nh_.param("YAML_FILE_NAME",yaml_file_name,{std::string("semi_dynamic_objects")});
        XmlRpc::XmlRpcValue semi_dynamic_objects;
        if(!private_nh_.getParam(yaml_file_name.c_str(),semi_dynamic_objects)){
            ROS_WARN("Cloud not load %s", yaml_file_name.c_str());
            return;
        }

        ROS_ASSERT(semi_dynamic_objects.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for(int i = 0; i < (int)semi_dynamic_objects.size(); i++){
            if(!semi_dynamic_objects[i]["name"].valid() ||
               !semi_dynamic_objects[i]["r"].valid() ||
               !semi_dynamic_objects[i]["g"].valid() ||
               !semi_dynamic_objects[i]["b"].valid()){
                ROS_WARN("%s is valid", yaml_file_name.c_str());
                return;
            }
            if(semi_dynamic_objects[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
               semi_dynamic_objects[i]["r"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
               semi_dynamic_objects[i]["g"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
               semi_dynamic_objects[i]["b"].getType() == XmlRpc::XmlRpcValue::TypeDouble){
                std::string name = static_cast<std::string>(semi_dynamic_objects[i]["name"]);
                double r = static_cast<double>(semi_dynamic_objects[i]["r"]);
                double g = static_cast<double>(semi_dynamic_objects[i]["g"]);
                double b = static_cast<double>(semi_dynamic_objects[i]["b"]);
                this->emplace_back(SemiDynamicObject(name,r,g,b));
            }
        }
    }

    std_msgs::ColorRGBA set_color_msg(double r,double g,double b)
    {
        std_msgs::ColorRGBA color_msg;
        color_msg.r = r;
        color_msg.g = g;
        color_msg.b = b;
        color_msg.a = 1.0;

        return color_msg;
    }

    ros::NodeHandle private_nh_;
};
}

#endif  // SEMI_DYNAMIC_OBJECTS_H_
