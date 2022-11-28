#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

// Custom msg
#include "object_detector_msgs/ObjectPositions.h"
#include "multi_robot_msgs/ObjectsData.h"

namespace multi_localizer
{
class AMCLPoseRepublisher
{
public:
    AMCLPoseRepublisher();
    void process();

private:
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void obj_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);

    void filter_ops_msg(object_detector_msgs::ObjectPositions input_ops,
                        object_detector_msgs::ObjectPositions& output_ops);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber obj_sub_;

    // publisher
    ros::Publisher pose_pub_;
    ros::Publisher obj_pub_;

    // tf
    boost::shared_ptr<tf2_ros::Buffer> buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> listener_;
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // buffer
    ros::Time start_time_;
    nav_msgs::Odometry odom_;
    geometry_msgs::PoseStamped pose_;

    // parameter
    std::string ROBOT_NAME_;
    std::string MAP_FRAME_ID_;
    std::string BASE_LINK_FRAME_ID_;
    bool PUBLISH_OBJ_MSG_;
    double PROBABILITY_TH_;
    double ANGLE_OF_VIEW_;
    double VISIBLE_LOWER_DISTANCE_;
    double VISIBLE_UPPER_DISTANCE_;
};
}

multi_localizer::AMCLPoseRepublisher::AMCLPoseRepublisher() :
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
    obj_sub_ = nh_.subscribe("obj_in",1,&AMCLPoseRepublisher::obj_callback,this);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_out",1);

    private_nh_.param("PUBLISH_OBJ_MSG",PUBLISH_OBJ_MSG_,{false});
    if(PUBLISH_OBJ_MSG_){
        obj_pub_ = nh_.advertise<multi_robot_msgs::ObjectsData>("obj_out",1);
    }

    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));
    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

void multi_localizer::AMCLPoseRepublisher::odom_callback(const nav_msgs::OdometryConstPtr& msg) { odom_ = *msg; }

void multi_localizer::AMCLPoseRepublisher::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    pose_.header = msg->header;
    pose_.pose = msg->pose.pose;
    pose_pub_.publish(pose_);

    tf2::Quaternion q;
    q.setRPY(0.0,0.0,tf2::getYaw(pose_.pose.orientation));
    tf2::Transform map_to_robot(q,tf2::Vector3(pose_.pose.position.x,pose_.pose.position.y,0.0));

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

void multi_localizer::AMCLPoseRepublisher::obj_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
    if(PUBLISH_OBJ_MSG_){
        multi_robot_msgs::ObjectsData data;

        // credibility
        data.credibility = 1.0;

        // header
        ros::Time now_time = ros::Time::now();
        data.header.frame_id = MAP_FRAME_ID_;
        data.header.stamp = now_time;

        // pose
        data.pose.name = ROBOT_NAME_;
        data.pose.weight = 1.0;
        data.pose.x = pose_.pose.position.x;
        data.pose.y = pose_.pose.position.y;
        data.pose.yaw = tf2::getYaw(pose_.pose.orientation);

        object_detector_msgs::ObjectPositions filtered_ops;
        filter_ops_msg(*msg,filtered_ops);
        std::cout << msg->object_position.size() << ","
                  << filtered_ops.object_position.size() << std::endl;
        if(filtered_ops.object_position.empty()) return;

        // objects
        for(const auto &m : filtered_ops.object_position){
            double dist = std::sqrt(m.x*m.x + m.z*m.z);
            double angle = std::atan2(m.z,m.x) - 0.5*M_PI;

            multi_robot_msgs::ObjectData object;
            object.name = m.Class;
            object.time = (now_time - start_time_).toSec();
            object.x = pose_.pose.position.x + dist*std::cos(tf2::getYaw(pose_.pose.orientation) + angle);

            object.y = pose_.pose.position.y + dist*std::sin(tf2::getYaw(pose_.pose.orientation) + angle);
            data.objects.emplace_back(object);
        }
        obj_pub_.publish(data);
    }
}

void multi_localizer::AMCLPoseRepublisher::filter_ops_msg(object_detector_msgs::ObjectPositions input_ops,
                                                          object_detector_msgs::ObjectPositions& output_ops)
{
    output_ops.header = input_ops.header;
    output_ops.object_position.clear();

    auto is_visible_range = [this](object_detector_msgs::ObjectPosition op) -> bool
    {
        if(op.Class == "roomba") return false;
        if(op.Class == "fire_extinguisher") return false;
        if(op.probability < PROBABILITY_TH_) return false;

        double r_vertex_x = std::cos(0.5*(M_PI - ANGLE_OF_VIEW_));
        double r_vertex_y = std::sin(0.5*(M_PI - ANGLE_OF_VIEW_));
        double l_vertex_x = std::cos(0.5*(M_PI + ANGLE_OF_VIEW_));
        double l_vertex_y = std::sin(0.5*(M_PI + ANGLE_OF_VIEW_));

        double dist = std::sqrt(op.x*op.x + op.z*op.z);
        if(VISIBLE_LOWER_DISTANCE_ < dist &&
           dist < VISIBLE_UPPER_DISTANCE_){
            double x = op.x;
            double y = op.z;
            if(r_vertex_x*y - x*r_vertex_y >= 0 && l_vertex_x*y - x*l_vertex_y <= 0) return true;
        }
        return false;
    };

    for(const auto &inp_op : input_ops.object_position){
        if(is_visible_range(inp_op)) output_ops.object_position.emplace_back(inp_op);
    }
}

void multi_localizer::AMCLPoseRepublisher::process() { ros::spin(); }

int main(int argc,char** argv)
{
    ros::init(argc,argv,"amcl_pose");
    multi_localizer::AMCLPoseRepublisher amcl_pose_republisher;
    amcl_pose_republisher.process();
    return 0;
}
