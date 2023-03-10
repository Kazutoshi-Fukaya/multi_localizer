#ifndef OBJECT_MAP_H_
#define OBJECT_MAP_H_

// ros
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>

// Custom msg
// #include "multi_localizer_msgs/ObjectMap.msg"
#include "multi_localizer_msgs/ObjectMap.h"

// c++
#include <map>
#include <sstream>
#include <fstream>

#include "ros_utils/object_map/objects.h"
#include "ros_utils/object_params/object_param.h"

namespace object_localizer
{
class ObjectMap : public std::map<ObjectParam*,Objects*>
{
public:
    ObjectMap();
    ObjectMap(ros::NodeHandle nh,ros::NodeHandle private_nh);

    void add_object(std::string name,Object object);
    void all_objects_are_not_observed();
    void publish_markers();
    void print_object_params(); // for debug
    void print_elements();      // for debug

    Object get_nearest_object(std::string name,double x,double y,double& sim);

private:
    void object_map_callback(const multi_localizer_msgs::ObjectMapConstPtr& msg);
    void load_object_params(ros::NodeHandle private_nh);
    void load_init_objects(ros::NodeHandle private_nh);

    visualization_msgs::Marker get_init_marker();
    geometry_msgs::Vector3 get_scale(double x,double y,double z);
    geometry_msgs::Pose get_pose(double x,double y);
    std_msgs::ColorRGBA get_color_msg(Color color);
    std::vector<std::string> split(std::string& input,char delimiter);

    // publisher
    ros::Publisher markers_pub_;

    // subscriber
    ros::Subscriber object_map_sub_;

    // param
    std::string MAP_FRAME_ID_;
};
} // namespace object_localizer

#endif  // OBJECT_MAP_H_
