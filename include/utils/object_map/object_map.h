#ifndef OBJECT_MAP_H_
#define OBJECT_MAP_H_

// ros
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>

// c++
#include <map>
#include <sstream>
#include <fstream>

// utils
#include "utils/object_param/object_param.h"
#include "utils/object_map/objects.h"

// Custom msg
#include "multi_robot_msgs/ObjectMap.h"

namespace multi_localizer
{
class ObjectMap : public std::map<ObjectParam*,Objects*>
{
public:
    ObjectMap();
    ObjectMap(ros::NodeHandle _nh,ros::NodeHandle _private_nh);

    void add_init_object(std::string name,Object object);
    void publish_markers();
    void load_init_objects();
    void all_objects_are_not_observed();
    Object get_highly_similar_object(std::string name,double x,double y,double& sim);

    // for debug
    void print_object_params();
    void print_elements();


private:
    void object_map_callback(const multi_robot_msgs::ObjectMapConstPtr& msg);
    
    void init();
    void load_object_params();
    void init_marker(visualization_msgs::Marker& marker);
    void clear_objects_data();
    geometry_msgs::Vector3 get_scale(double x,double y,double z);
    geometry_msgs::Pose get_pose(double x,double y);
    std::vector<std::string> split(std::string& input,char delimiter);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // publisher
    ros::Publisher markers_pub_;

    // subscriber
    ros::Subscriber object_map_sub_;

    // parameters
    std::string MAP_FRAME_ID_;
};
} // namespace multi_localizer

#endif  // OBJECT_MAP_H_
