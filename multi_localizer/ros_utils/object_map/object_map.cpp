#include "ros_utils/object_map/object_map.h"

using namespace multi_localizer;

ObjectMap::ObjectMap()
{
    ROS_WARN("'object_map' has not received 'nh' and 'private_nh'");
}

ObjectMap::ObjectMap(ros::NodeHandle nh,ros::NodeHandle private_nh)
{
    private_nh.param("MAP_FRAME_ID",MAP_FRAME_ID_,std::string("map"));

    object_map_sub_ = nh.subscribe("object_map_in",1,&ObjectMap::object_map_callback,this);
    markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("markers_out",1);

    load_object_params(private_nh);
    load_init_objects(private_nh);
}

void ObjectMap::object_map_callback(const multi_localizer_msgs::ObjectMapConstPtr& msg)
{
    if(msg->data.empty()) return;
    for(auto it = this->begin(); it != this->end(); it++) it->second->clear();    
    for(const auto &data : msg->data){
        this->add_init_object(data.name,Object(data.time,data.credibility,data.x,data.y));
    }
}

void ObjectMap::load_object_params(ros::NodeHandle private_nh)
{
    this->clear();
    std::string yaml_file_name;
    private_nh.param("YAML_FILE_NAME",yaml_file_name,{std::string("object_params")});
    XmlRpc::XmlRpcValue object_params;
    if(!private_nh.getParam(yaml_file_name.c_str(),object_params)){
        ROS_WARN("Cloud not load %s", yaml_file_name.c_str());
        return;
    }

    ROS_ASSERT(object_params.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < object_params.size(); i++){
        if(!object_params[i]["name"].valid() || !object_params[i]["condition"].valid() ||
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
    std::ifstream ifs(init_objects_file_name);
    std::string line;
    while(std::getline(ifs,line)){
        std::vector<std::string> strvec = split(line,',');
        try{
            std::string name = static_cast<std::string>(strvec[0]);
            double time = static_cast<double>(std::stod(strvec[1]));
            double x = static_cast<double>(std::stod(strvec[2]));
            double y = static_cast<double>(std::stod(strvec[3]));
            this->add_init_object(name,Object(time,1.0,x,y));
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

void ObjectMap::add_init_object(std::string name,Object object)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->first->name == name) it->second->add_init_object(object);
    }
}

void ObjectMap::clear_objects_data()
{
    for(auto it = this->begin(); it != this->end(); it++) it->second->clear();
}

void ObjectMap::all_objects_are_not_observed()
{
    for(auto it = this->begin(); it != this->end(); it++){
        for(auto sit = it->second->begin(); sit != it->second->end(); sit++){
            sit->has_observed = false;
        }
    }
}

void ObjectMap::publish_markers()
{
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    for(auto it = this->begin(); it != this->end(); it++){
        for(auto sit = it->second->begin(); sit != it->second->end(); sit++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = MAP_FRAME_ID_;
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.6;
            marker.id = marker_id;
            marker.ns = it->first->name;
            marker.pose.position.x = sit->x;
            marker.pose.position.y = sit->y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.color.r = it->first->color.r;
            marker.color.g = it->first->color.g;
            marker.color.b = it->first->color.b;
            if(sit->has_observed) marker.color.a = 1.0;
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

Object ObjectMap::get_highly_similar_object(std::string name,double x,double y,double& sim)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->first->name == name){
            return it->second->get_highly_similar_object(x,y,sim);
        }
    }
    ROS_ERROR("No corresponding object");
    sim = -1e10;
    return Object();
}

std::vector<std::string> ObjectMap::split(std::string& input,char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.push_back(field);
    return result;
}