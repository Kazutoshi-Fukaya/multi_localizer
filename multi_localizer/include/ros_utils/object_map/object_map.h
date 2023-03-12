#ifndef OBJECT_MAP_H_
#define OBJECT_MAP_H_

// ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// Custom msg
#include "multi_localizer_msgs/ObjectMap.h"

// c++
#include <map>
#include <sstream>
#include <fstream>

// ros_utils
#include "ros_utils/object_map/object_param.h"
#include "ros_utils/object_map/objects.h"

namespace multi_localizer
{
class ObjectMap : public std::map<ObjectParam*,Objects*>
{
public:
    ObjectMap();
    ObjectMap(ros::NodeHandle nh,ros::NodeHandle private_nh);

    void load_object_params(ros::NodeHandle private_nh);
    void load_init_objects(ros::NodeHandle private_nh);
    void add_init_object(std::string name,Object object);
    void clear_objects_data();
    void all_objects_are_not_observed();
    void publish_markers();
    void print_object_params(); // for debug
    void print_elements();      // for debug

    Object get_highly_similar_object(std::string name,double x,double y,double& sim);
    std::vector<std::string> split(std::string& input,char delimiter);

private:
    void object_map_callback(const multi_localizer_msgs::ObjectMapConstPtr& msg);

    // subscriber
    ros::Subscriber object_map_sub_;

    // publisher
    ros::Publisher markers_pub_;

    // params
    std::string MAP_FRAME_ID_;
};
} // namespace multi_localizer

#endif  // OBJECT_MAP_H_
