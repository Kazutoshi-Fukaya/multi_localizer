#include "ros_utils/object_map/object_map.h"

using namespace object_localizer;

ObjectMap::ObjectMap() 
{
	ROS_WARN("'object map' has not received 'nh' and 'private_nh'");
}

ObjectMap::ObjectMap(ros::NodeHandle nh,ros::NodeHandle private_nh)
{
	private_nh.param("MAP_FRAME_ID",MAP_FRAME_ID_,{std::string("map")});
	
	object_map_sub_ = nh.subscribe("object_map",1,&ObjectMap::object_map_callback,this);
    markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("objects",1);

    load_object_params(private_nh);
    load_init_objects(private_nh);
}

void ObjectMap::object_map_callback(const multi_localizer_msgs::ObjectMapConstPtr& msg)
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->second->clear();
    }
    for(const auto &data : msg->data){
        add_object(data.name,Object(data.time,data.credibility,data.x,data.y));
    }
}

void ObjectMap::load_object_params(ros::NodeHandle private_nh)
{
	std::string yaml_file_name;
	private_nh.param("YAML_FILE_NAME",yaml_file_name,{std::string("object_params")});
	XmlRpc::XmlRpcValue object_params;
	if(!private_nh.getParam(yaml_file_name.c_str(),object_params)){
		ROS_WARN("Cloud not load %s", yaml_file_name.c_str());
        return;
    }
	
	ROS_ASSERT(object_params.getType() == XmlRpc::XmlRpcValue::TypeArray);
	for(int i = 0; i < object_params.size(); i++){
		if(!object_params[i]["name"].valid() ||
		   !object_params[i]["condition"].valid() ||
		   !object_params[i]["r"].valid() || !object_params[i]["g"].valid() || !object_params[i]["b"].valid()){
			ROS_WARN("%s is valid", yaml_file_name.c_str());
            return;
        }
		if(object_params[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
		   object_params[i]["condition"].getType() == XmlRpc::XmlRpcValue::TypeString &&
           object_params[i]["r"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
           object_params[i]["g"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
           object_params[i]["b"].getType() == XmlRpc::XmlRpcValue::TypeDouble){
			std::string name = static_cast<std::string>(object_params[i]["name"]);
            std::string condition = static_cast<std::string>(object_params[i]["condition"]);
            double r = static_cast<double>(object_params[i]["r"]);
            double g = static_cast<double>(object_params[i]["g"]);
            double b = static_cast<double>(object_params[i]["b"]);
			ObjectParam* object_param (new ObjectParam(name,condition,Color(r,g,b)));
            Objects* objects (new Objects());
            this->insert(std::map<ObjectParam*,Objects*>::value_type(object_param,objects));
        }
    }
}

void ObjectMap::load_init_objects(ros::NodeHandle private_nh)
{
    std::string init_objects_file_name;
    private_nh.param("INIT_OBJECTS_FILE_NAME",init_objects_file_name,{std::string("")});
    static std::ifstream ifs(init_objects_file_name);
    std::string line;
    while(std::getline(ifs,line)){
        std::vector<std::string> strvec = split(line,',');
        try{
            std::string name = static_cast<std::string>(strvec[0]);
            double time = static_cast<double>(std::stod(strvec[1]));
            double x = static_cast<double>(std::stod(strvec[2]));
            double y = static_cast<double>(std::stod(strvec[3]));
            add_object(name,Object(time,1.0,x,y));
        }
        catch(const std::invalid_argument& ex){
            ROS_ERROR("invalid: %s", ex.what());
        }
        catch(const std::out_of_range& ex){
            ROS_ERROR("out of range: %s", ex.what());
        }
    }
    ifs.close();
}

void ObjectMap::add_object(std::string name,Object object)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->first->name == name) it->second->add_object(object);
    }
}

void ObjectMap::all_objects_are_not_observed()
{
    for(auto it = this->begin(); it != this->end(); it++){
        for(auto sit = it->second->begin(); sit != it->second->end(); sit++){
            sit->is_observed = false;
        }
    }
}

void ObjectMap::publish_markers()
{
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    for(auto it = this->begin(); it != this->end(); it++){
        for(auto sit = it->second->begin(); sit != it->second->end(); sit++){
            visualization_msgs::Marker marker = get_init_marker();
            marker.id = marker_id;
            marker.ns = it->first->name;
            marker.pose = get_pose(sit->x,sit->y);
            marker.color = get_color_msg(it->first->color);
            if(sit->is_observed) marker.color.a = 1.0;
            else marker.color.a = 0.3;
            markers.markers.emplace_back(marker);
            marker_id++;
        }
    }
    markers_pub_.publish(markers);
}

void ObjectMap::print_object_params()
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->first->print_element();
    }
}

void ObjectMap::print_elements()
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->first->print_param();
        it->second->print_elements();
    }
}

Object ObjectMap::get_nearest_object(std::string name,double x,double y,double& sim)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->first->name == name){
            return it->second->get_nearest_object(x,y,sim);
        }
    }
    ROS_ERROR("No corresponding object");
    sim = -1e10;
    return Object();
}

visualization_msgs::Marker ObjectMap::get_init_marker()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = MAP_FRAME_ID_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker.scale =  get_scale(0.4,0.4,0.6);
    return marker;
}

geometry_msgs::Vector3 ObjectMap::get_scale(double x,double y,double z)
{
    geometry_msgs::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    return scale;
}

geometry_msgs::Pose ObjectMap::get_pose(double x,double y)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}

std_msgs::ColorRGBA ObjectMap::get_color_msg(Color color)
{
    std_msgs::ColorRGBA color_msg;
    color_msg.r = color.r;
    color_msg.g = color.g;
    color_msg.b = color.b;
    color_msg.a = 0.0;
    return color_msg;
}

std::vector<std::string> ObjectMap::split(std::string& input,char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.push_back(field);
    return result;
}