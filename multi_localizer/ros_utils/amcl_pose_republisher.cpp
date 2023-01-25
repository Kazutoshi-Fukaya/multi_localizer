#include "ros_utils/amcl_pose_republisher/amcl_pose_republisher.h"

using namespace multi_localizer;

AMCLPoseRepublisher::AMCLPoseRepublisher() :
    private_nh_("~"), start_time_(ros::Time::now())
{
    private_nh_.param("ROBOT_NAME",ROBOT_NAME_,{std::string("")});
    private_nh_.param("MAP_FRAME_ID",MAP_FRAME_ID_,{std::string("map")});
    private_nh_.param("BASE_LINK_FRAME_ID",BASE_LINK_FRAME_ID_,{std::string("base_link")});

    private_nh_.param("PROBABILITY_TH",PROBABILITY_TH_,{0.8});
    private_nh_.param("ANGLE_OF_VIEW",ANGLE_OF_VIEW_,{86.0/180.0*M_PI});
    private_nh_.param("VISIBLE_LOWER_DISTANCE",VISIBLE_LOWER_DISTANCE_,{0.1});
    private_nh_.param("VISIBLE_UPPER_DISTANCE",VISIBLE_UPPER_DISTANCE_,{5.0});

    pose_sub_ = nh_.subscribe("pose_in",1,&AMCLPoseRepublisher::pose_callback,this);
    odom_sub_ = nh_.subscribe("odom_in",1,&AMCLPoseRepublisher::odom_callback,this);
    od_sub_ = nh_.subscribe("od_in",1,&AMCLPoseRepublisher::od_callback,this);

    pose_pub_ = nh_.advertise<multi_localizer_msgs::RobotPoseStamped>("pose_out",1);

    private_nh_.param("PUBLISH_OBJECTS_DATA",PUBLISH_OBJECTS_DATA_,{false});
    if(PUBLISH_OBJECTS_DATA_){
        data_pub_ = nh_.advertise<multi_localizer_msgs::ObjectsData>("data_out",1);
    }

    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));
    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

void AMCLPoseRepublisher::odom_callback(const nav_msgs::OdometryConstPtr& msg) { odom_ = *msg; }

void AMCLPoseRepublisher::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    pose_.header = msg->header;
    pose_.pose.name = ROBOT_NAME_;
    pose_.pose.weight = 1.0;
    pose_.pose.x = msg->pose.pose.position.x;
    pose_.pose.y = msg->pose.pose.position.y;
    pose_.pose.theta = tf2::getYaw(msg->pose.pose.orientation);
    pose_pub_.publish(pose_);

    tf2::Quaternion q;
    q.setRPY(0.0,0.0,tf2::getYaw(msg->pose.pose.orientation));
    tf2::Transform map_to_robot(q,tf2::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,0.0));

    geometry_msgs::PoseStamped robot_to_map_pose;
    robot_to_map_pose.header.frame_id = BASE_LINK_FRAME_ID_;
    robot_to_map_pose.header.stamp = odom_.header.stamp;
    tf2::toMsg(map_to_robot.inverse(),robot_to_map_pose.pose);

    geometry_msgs::PoseStamped odom_to_map_pose;
    try{
        buffer_->transform(robot_to_map_pose,odom_to_map_pose,odom_.header.frame_id);
    }
    catch(tf2::TransformException& ex){
        ROS_WARN("%s", ex.what());
        return;
    }
    tf2::Transform odom_to_map;
    tf2::convert(odom_to_map_pose.pose,odom_to_map);
    geometry_msgs::TransformStamped map_to_odom_transform;
    map_to_odom_transform.header.stamp = odom_.header.stamp;
    map_to_odom_transform.header.frame_id = MAP_FRAME_ID_;
    map_to_odom_transform.child_frame_id = odom_.header.frame_id;
    tf2::convert(odom_to_map.inverse(),map_to_odom_transform.transform);
    broadcaster_->sendTransform(map_to_odom_transform);
}

void AMCLPoseRepublisher::od_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
    if(PUBLISH_OBJECTS_DATA_){
        multi_localizer_msgs::ObjectsData objects;
        ros::Time now_time = ros::Time::now();
        objects.header.frame_id = MAP_FRAME_ID_;
        objects.header.stamp = now_time;
        objects.pose = pose_.pose;

        object_detector_msgs::ObjectPositions filtered_od;
        filter_od(*msg,filtered_od);
        if(filtered_od.object_position.empty()) return;

        // objects
        for(const auto &p : filtered_od.object_position){
            double dist = std::sqrt(p.x*p.x + p.z*p.z);
            double angle = std::atan2(p.z,p.x) - 0.5*M_PI;

            multi_localizer_msgs::ObjectData object;
            object.name = p.Class;
            object.credibility = pose_.pose.weight;
            object.time = (now_time - start_time_).toSec();
            object.x = pose_.pose.x + dist*std::cos(pose_.pose.theta + angle);
            object.y = pose_.pose.x + dist*std::sin(pose_.pose.theta + angle);
            objects.data.emplace_back(object);
        }
        data_pub_.publish(objects);
    }
}

void AMCLPoseRepublisher::filter_od(object_detector_msgs::ObjectPositions input_od,object_detector_msgs::ObjectPositions& output_od)
{
    output_od.header = input_od.header;
    output_od.object_position.clear();

    for(const auto &p : input_od.object_position){
        if(is_visible_range(p)) output_od.object_position.emplace_back(p);
    }
}

bool AMCLPoseRepublisher::is_visible_range(object_detector_msgs::ObjectPosition od)
{
    if(od.Class == "roomba") return false;
    if(od.Class == "fire_extinguisher") return false;
    if(od.probability < PROBABILITY_TH_) return false;

    double r_vertex_x = std::cos(0.5*(M_PI - ANGLE_OF_VIEW_));
    double r_vertex_y = std::sin(0.5*(M_PI - ANGLE_OF_VIEW_));
    double l_vertex_x = std::cos(0.5*(M_PI + ANGLE_OF_VIEW_));
    double l_vertex_y = std::sin(0.5*(M_PI + ANGLE_OF_VIEW_));
    
    double dist = std::sqrt(od.x*od.x + od.z*od.z);
    if(VISIBLE_LOWER_DISTANCE_ < dist && dist < VISIBLE_UPPER_DISTANCE_){
        double x = od.x;
        double y = od.z;
        if(r_vertex_x*y - x*r_vertex_y >= 0 && l_vertex_x*y - x*l_vertex_y <= 0) return true;
    }
    return false;
}

void AMCLPoseRepublisher::process() { ros::spin(); }

int main(int argc,char** argv)
{
    ros::init(argc,argv,"amcl_pose_republisher");
    AMCLPoseRepublisher amcl_pose_republisher;
    amcl_pose_republisher.process();
    return 0;
}
