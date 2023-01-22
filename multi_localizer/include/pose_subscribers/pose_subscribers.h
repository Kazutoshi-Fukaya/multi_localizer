#ifndef POSE_SUBSCRIBERS_H_
#define POSE_SUBSCRIBERS_H_

#include <ros/ros.h>
#include <tf2/utils.h>

#include <map>

#include "pose_subscribers/pose_subscriber.h"
#include "pose_subscribers/robot_element.h"

class PoseSubscribers : public std::map<RobotElement*,PoseSubscriber*>
{
public:
    PoseSubscribers(ros::NodeHandle _nh,ros::NodeHandle _private_nh) :
        nh_(_nh), private_nh_(_private_nh) { load_robot_element(); }

    void set_received_flag(std::string color,bool flag)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->first->color == color)  return it->second->set_received_flag(flag);
        }
    }

    std::string get_robot_name(std::string color)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->first->color == color)  return it->first->robot_name;
        }
        return std::string("");
    }

    std::string get_color(std::string robot_name)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->first->robot_name == robot_name) return it->first->color;
        }
        return std::string("");
    }

    bool get_pose(std::string color,geometry_msgs::PoseStamped& pose)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->first->color ==  color) return it->second->get_pose(pose);
        }
        return false;
    }

    void debug()
    {
        std::string color = "GREEN";
        geometry_msgs::PoseStamped pose;

        if(get_pose(color,pose)){
            std::cout << " pose.x : " << pose.pose.position.x << std::endl;
            std::cout << " pose.y : " << pose.pose.position.y << std::endl;
            std::cout << "pose.yaw: " << tf2::getYaw(pose.pose.orientation) << std::endl;
        }
        else{
            std::cout << "No data" << std::endl;
        }
    }

private:
    void load_robot_element()
    {
        this->clear();
        std::string robot_element_list_name;
        private_nh_.param("ROBOT_ELEMENT_LIST",robot_element_list_name,{std::string("robot_element_list")});
        XmlRpc::XmlRpcValue robot_element_list;
        if(!private_nh_.getParam(robot_element_list_name.c_str(),robot_element_list)){
            ROS_ERROR("Cloud not load %s", robot_element_list_name.c_str());
            return;
        }

        ROS_ASSERT(robot_element_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for(int i = 0; i < (int)robot_element_list.size(); i++){
            if(!robot_element_list[i]["robot_name"].valid() ||
               !robot_element_list[i]["color"].valid()){
                ROS_ERROR("%s is valid", robot_element_list_name.c_str());
                return;
            }
            if(robot_element_list[i]["robot_name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
               robot_element_list[i]["color"].getType() == XmlRpc::XmlRpcValue::TypeString){
                std::string robot_name = static_cast<std::string>(robot_element_list[i]["robot_name"]);
                std::string color = static_cast<std::string>(robot_element_list[i]["color"]);
                std::string  topic_name = robot_name + "/pose";
                RobotElement* robot_element (new RobotElement(robot_name,color));
                PoseSubscriber* pose_subscriber (new PoseSubscriber(nh_,topic_name));
                this->insert(std::map<RobotElement*,PoseSubscriber*>::value_type(robot_element,pose_subscriber));
            }
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
};

#endif  // POSE_SUBSCRIBERS_H_
