#include "ros_utils/pose_subscribers/pose_subscribers.h"

using namespace multi_localizer;

PoseSubscribers::PoseSubscribers()
{
	ROS_WARN("'pose_subscribers' has not received 'nh' and 'private_nh'");
}

PoseSubscribers::PoseSubscribers(ros::NodeHandle nh,ros::NodeHandle private_nh)
{
	load_robot_element(nh,private_nh); 
}

void PoseSubscribers::load_robot_element(ros::NodeHandle nh,ros::NodeHandle private_nh)
{
	this->clear();
    std::string robot_element_list_name;
    private_nh.param("ROBOT_ELEMENT_LIST",robot_element_list_name,{std::string("robot_element_list")});
    XmlRpc::XmlRpcValue robot_element_list;
    if(!private_nh.getParam(robot_element_list_name.c_str(),robot_element_list)){
		ROS_ERROR("Cloud not load %s", robot_element_list_name.c_str());
        return;
    }

    ROS_ASSERT(robot_element_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < (int)robot_element_list.size(); i++){
        if(!robot_element_list[i]["robot_name"].valid() || !robot_element_list[i]["color"].valid()){
            ROS_ERROR("%s is valid", robot_element_list_name.c_str());
            return;
        }
        if(robot_element_list[i]["robot_name"].getType() == XmlRpc::XmlRpcValue::TypeString && robot_element_list[i]["color"].getType() == XmlRpc::XmlRpcValue::TypeString){
            std::string robot_name = static_cast<std::string>(robot_element_list[i]["robot_name"]);
            std::string color = static_cast<std::string>(robot_element_list[i]["color"]);
            std::string  topic_name = robot_name + "/robot_pose";
            RobotElement* robot_element (new RobotElement(robot_name,color));
            PoseSubscriber* pose_subscriber (new PoseSubscriber(nh,topic_name));
            this->insert(std::map<RobotElement*,PoseSubscriber*>::value_type(robot_element,pose_subscriber));
        }
    }
}

void PoseSubscribers::set_received_flag(std::string color,bool flag)
{
	for(auto it = this->begin(); it != this->end(); it++){
		if(it->first->color == color){
			return it->second->set_received_flag(flag);
        }
    }
}

void PoseSubscribers::debug(std::string color)
{
	multi_localizer_msgs::RobotPoseStamped pose;
	if(get_pose(color,pose)){
		std::cout << "POSE(X,Y,YAW): ("
		          << pose.pose.x << ","
				  << pose.pose.y << ","
				  << pose.pose.theta << ")" << std::endl;
    }
    else{
        std::cout << "No data" << std::endl;
    }
}

std::string PoseSubscribers::get_robot_name(std::string color)
{
	for(auto it = this->begin(); it != this->end(); it++){
		if(it->first->color == color){
			return it->first->robot_name;
        }
	}
    return std::string("");
}

std::string PoseSubscribers::get_color(std::string robot_name)
{
	for(auto it = this->begin(); it != this->end(); it++){
		if(it->first->robot_name == robot_name){
			return it->first->color;
        }
	}
    return std::string("");
}

bool PoseSubscribers::get_pose(std::string color,multi_localizer_msgs::RobotPoseStamped& pose)
{
	for(auto it = this->begin(); it != this->end(); it++){
		if(it->first->color ==  color){
			return it->second->get_pose(pose);
        }
	}
    return false;
}