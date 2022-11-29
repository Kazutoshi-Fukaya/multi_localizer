#include "dynamic_mcl/semi_dynamic_map_creator.h"

using namespace multi_localizer;

SemiDynamicMapCreator::SemiDynamicMapCreator() :
    private_nh_("~"),
    has_received_pose_(false), has_received_map_(false),
    semi_dynamic_objects_(new SemiDynamicObjects(private_nh_))
{
    private_nh_.param("LOCAL_MAP_FRAME_ID",LOCAL_MAP_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("HAS_REMOVED_POLE",HAS_REMOVED_POLE_,{true});
    private_nh_.param("WIDTH",WIDTH_,{5});
    private_nh_.param("HEIGHT",HEIGHT_,{5});
    private_nh_.param("RANGE_STEP",RANGE_STEP_,{5});
    private_nh_.param("RESOLUTION",RESOLUTION_,{0.05});
    private_nh_.param("ROBOT_RADIUS",ROBOT_RADIUS_,{0.2});
    private_nh_.param("POLE_MARGIN",POLE_MARGIN_,{0.2});
    private_nh_.param("PROBABILITY_TH",PROBABILITY_TH_,{0.8});
    private_nh_.param("VISIBLE_LOWER_DISTANCE",VISIBLE_LOWER_DISTANCE_,{0.2});
    private_nh_.param("VISIBLE_UPPER_DISTANCE",VISIBLE_UPPER_DISTANCE_,{5.0});

    lsr_sub_ = nh_.subscribe("lsr_in",1,&SemiDynamicMapCreator::lsr_callback,this);
    pose_sub_ = nh_.subscribe("pose_in",1,&SemiDynamicMapCreator::pose_callback,this);
    obj_sub_ = nh_.subscribe("obj_in",1,&SemiDynamicMapCreator::obj_callback,this);
    map_sub_ = nh_.subscribe("map_in",1,&SemiDynamicMapCreator::map_callback,this);

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_out",1);
    latest_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("latest_map_out",1);
    obs_pub_ = nh_.advertise<geometry_msgs::PoseArray>("obs_out",1);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obj_out",1);

    obstacle_poses_.header.frame_id = LOCAL_MAP_FRAME_ID_;
    generate_map();
}

void SemiDynamicMapCreator::lsr_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    lsr_ = *msg;
    init_map();
    update_map();
    map_pub_.publish(local_map_);
    obs_pub_.publish(obstacle_poses_);
}

void SemiDynamicMapCreator::pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    pose_ = *msg;
    has_received_pose_ = true;
}

void SemiDynamicMapCreator::obj_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
    if(has_received_pose_){
        object_detector_msgs::ObjectPositions filtered_msg;
        filter_ops_msg(*msg,filtered_msg);
        if(filtered_msg.object_position.empty()) return;
        visualization_msgs::MarkerArray markers;
        int marker_id = 0;
        for(const auto &obj_pos : filtered_msg.object_position){
            visualization_msgs::Marker marker;
            // setup
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.type = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();
            marker.id = marker_id;
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.1;

            // obj info
            double dist = std::sqrt(obj_pos.x*obj_pos.x + obj_pos.z*obj_pos.z);
            double angle = std::atan2(obj_pos.z,obj_pos.x) - 0.5*M_PI;
            marker.ns = obj_pos.Class;
            marker.pose.position.x = pose_.pose.position.x + dist*std::cos(tf2::getYaw(pose_.pose.orientation) + angle);
            marker.pose.position.y = pose_.pose.position.y + dist*std::sin(tf2::getYaw(pose_.pose.orientation) + angle);
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.color = semi_dynamic_objects_->get_color(obj_pos.Class);
            marker.color.a = obj_pos.probability;
            markers.markers.emplace_back(marker);
            marker_id++;
        }
        update_latest_map(markers,obstacle_poses_);
        markers_pub_.publish(markers);
        has_received_pose_ = false;
    }
    latest_map_pub_.publish(latest_map_);
}

void SemiDynamicMapCreator::map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if(!has_received_map_){
        latest_map_ = *msg;
        has_received_map_ = true;
    }
}

void SemiDynamicMapCreator::generate_map()
{
    local_map_.header.frame_id = LOCAL_MAP_FRAME_ID_;
    local_map_.info.resolution = RESOLUTION_;
    local_map_.info.width = (int)(WIDTH_/RESOLUTION_);
    local_map_.info.height = (int)(HEIGHT_/RESOLUTION_);
    local_map_.info.origin.position.x = -WIDTH_/2;
    local_map_.info.origin.position.y = -HEIGHT_/2;
    local_map_.data.resize(local_map_.info.width*local_map_.info.height);
    init_map();
}

void SemiDynamicMapCreator::init_map()
{
    local_map_.data.clear();
    for(int i = 0; i < (int)(local_map_.info.width*local_map_.info.height); i++){
        local_map_.data.emplace_back(-1);
    }
}

void SemiDynamicMapCreator::update_map()
{
    obstacle_poses_.poses.clear();
    for(size_t i = 0; i < lsr_.ranges.size(); i += RANGE_STEP_){
        double angle = i*lsr_.angle_increment + lsr_.angle_min;
        input_to_map(angle,lsr_.ranges[i]);
    }
}

void SemiDynamicMapCreator::input_to_map(double angle,double lsr_range)
{
    if(!HAS_REMOVED_POLE_){
        if(lsr_range <= ROBOT_RADIUS_ || is_pole(angle)) lsr_range = WIDTH_;
    }

    for(double d = 0.0; d < WIDTH_; d += RESOLUTION_){
        double search_x = d*std::cos(angle);
        double search_y = d*std::sin(angle);

        if(!is_in_map(search_x,search_y)) return;

        if(d >= lsr_range){
            local_map_.data[coordinate_to_map_index(search_x,search_y)] = 100;
            geometry_msgs::Pose obstacle_pose;
            obstacle_pose.position.x = search_x;
            obstacle_pose.position.y = search_y;
            obstacle_poses_.poses.emplace_back(obstacle_pose);
            return;
        }
        else{
            local_map_.data[coordinate_to_map_index(search_x,search_y)] = 0;
        }
    }
}

void SemiDynamicMapCreator::filter_ops_msg(object_detector_msgs::ObjectPositions input_ops,
                                           object_detector_msgs::ObjectPositions& output_ops)
{
    output_ops.header = input_ops.header;
    output_ops.object_position.clear();

    auto is_visible_range = [this](object_detector_msgs::ObjectPosition op) -> bool
    {
        if(!semi_dynamic_objects_->is_included(op.Class)) return false;
        if(op.probability <= PROBABILITY_TH_) return false;

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

void SemiDynamicMapCreator::update_latest_map(visualization_msgs::MarkerArray obj_poses,
                                              geometry_msgs::PoseArray obs_poses)
{
    for(const auto &obj_pos : obj_poses.markers){
        // geometry_msgs::PoseArray obs_poses = get_obs(obj_pos.pose,obstacle_poses_);
        // std::cout << obs_poses.poses.size() << std::endl;
        // add_map_pose(obs_poses);
        double range = 0.2;
        double range_step = 0.01;
        for(double x = obj_pos.pose.position.x - range; x < obj_pos.pose.position.x + range; x += range_step){
            for(double y = obj_pos.pose.position.y - range; y < obj_pos.pose.position.y + range; y += range_step){
                int index_x = std::floor((x - latest_map_.info.origin.position.x)/latest_map_.info.resolution);
                int index_y = std::floor((y - latest_map_.info.origin.position.y)/latest_map_.info.resolution);
                int index = index_x + index_y*latest_map_.info.width;
                latest_map_.data[index] = 100;
            }
        }
    }
}

void SemiDynamicMapCreator::add_map_pose(geometry_msgs::PoseArray obs_poses)
{
    for(const auto &obs_pos : obs_poses.poses){
        int index_x = std::floor((obs_pos.position.x - latest_map_.info.origin.position.x)/latest_map_.info.resolution);
        int index_y = std::floor((obs_pos.position.y - latest_map_.info.origin.position.y)/latest_map_.info.resolution);
        int index = index_x + index_y*latest_map_.info.width;
        latest_map_.data[index] = 100;
    }
}

geometry_msgs::PoseArray SemiDynamicMapCreator::get_obs(geometry_msgs::Pose pose,
                                                        geometry_msgs::PoseArray poses)
{
    geometry_msgs::PoseArray obs_poses;
    // obs_poses.header = obstacle_poses_.header;
    for(size_t i = 0; i < poses.poses.size(); i++){
        double diff_x = poses.poses[i].position.x + pose_.pose.position.x - pose.position.x;
        double diff_y = poses.poses[i].position.y + pose_.pose.position.y - pose.position.y;
        double diff = std::sqrt(diff_x*diff_x + diff_y*diff_y);
        if(diff < 0.5) obs_poses.poses.emplace_back(poses.poses[i]);
    }
    return obs_poses;
}

bool SemiDynamicMapCreator::is_pole(double angle)
{
    // back left
    if(-0.75*M_PI <= angle && angle <= -0.75*M_PI + POLE_MARGIN_) return true;
    // front left
    else if(-0.25*M_PI - POLE_MARGIN_ <= angle && angle <= -0.25*M_PI + POLE_MARGIN_) return true;
    // front right
    else if(0.25*M_PI - POLE_MARGIN_ <= angle && angle <= 0.25*M_PI + POLE_MARGIN_) return true;
    // back right
    else if(0.75*M_PI - POLE_MARGIN_ <= angle && angle <= 0.75*M_PI) return true;

    // is not pole
    return false;
}

bool SemiDynamicMapCreator::is_in_map(double x,double y)
{
    double x_min = local_map_.info.origin.position.x;
    double x_max = x_min + local_map_.info.width*local_map_.info.resolution;
    double y_min = local_map_.info.origin.position.y;
    double y_max = y_min + local_map_.info.height*local_map_.info.resolution;

    if(x_min < x && x < x_max && y_min < y && y < y_max) return true;
    return false;
}

int SemiDynamicMapCreator::coordinate_to_map_index(double x,double y)
{
    int index_x = std::floor((x - local_map_.info.origin.position.x)/local_map_.info.resolution);
    int index_y = std::floor((y - local_map_.info.origin.position.y)/local_map_.info.resolution);

    return index_x + index_y*local_map_.info.width;
}

void SemiDynamicMapCreator::process() { ros::spin(); }

int main(int argc,char** argv)
{
    ros::init(argc,argv,"semi_dynamic_map_creator");
    SemiDynamicMapCreator semi_dynamic_map_creator;
    semi_dynamic_map_creator.process();
    return 0;
}
