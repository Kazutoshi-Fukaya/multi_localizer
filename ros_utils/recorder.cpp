#include "ros_utils/recorder/recorder.h"

using namespace multi_localizer;

Recorder::Recorder() : 
	private_nh_("~"), start_time_(ros::Time::now()) 
{
	private_nh_.param("HZ",HZ_,{1});
}

void Recorder::ref_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	ref_pose_.x = msg->pose.pose.position.x;
	ref_pose_.y = msg->pose.pose.position.y;
	ref_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
}

void Recorder::est_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	est_pose_.x = msg->pose.position.x;
	est_pose_.y = msg->pose.position.y;
	est_pose_.yaw = tf2::getYaw(msg->pose.orientation);
}

void Recorder::obs_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
	double time = get_time();
	for(const auto &p : msg->object_position){
		observations_.emplace_back(RecordObservation(get_time(),p.Class));
	}
}

void Recorder::add_trajectory()
{
	RecordTrajectory trajectory;
	trajectory.time = get_time();
    trajectory.est_pose = est_pose_;
    trajectory.ref_pose = ref_pose_;
    trajectories_.emplace_back(trajectory);
}

void Recorder::save_csv()
{
	std::string record_file_path;
	private_nh_.param("RECORD_FILE_PATH",record_file_path,{std::string("")});
	std::string file_name = record_file_path + "trajectory_" + get_date() + ".csv";
	std::ofstream traj_ofs(file_name);
	for(const auto &traj : trajectories_){
		traj_ofs << traj.time << ","
		         << traj.est_pose.x << "," << traj.est_pose.y << "," << traj.est_pose.yaw << ","
				 << traj.ref_pose.x << "," << traj.ref_pose.y << "," << traj.ref_pose.yaw << std::endl;
    }
    traj_ofs.close();
    std::cout << "\nSave: " << file_name << std::endl;

    file_name = record_file_path + "observation_" + get_date() + ".csv";
    std::ofstream obs_ofs(file_name);
    for(const auto &obs : observations_){
		obs_ofs << obs.time << "," << obs.name << std::endl;
    }
    obs_ofs.close();
    std::cout << "Save: " << file_name << std::endl;
}

std::string Recorder::get_date()
{
	time_t t = time(nullptr);
    const tm* localTime = localtime(&t);
    std::stringstream s;
    s << localTime->tm_year + 1900;
    s << std::setw(2) << std::setfill('0') << localTime->tm_mon + 1;
    s << std::setw(2) << std::setfill('0') << localTime->tm_mday;
    s << std::setw(2) << std::setfill('0') << localTime->tm_hour;
    s << std::setw(2) << std::setfill('0') << localTime->tm_min;
    s << std::setw(2) << std::setfill('0') << localTime->tm_sec;

    return s.str();
}

double Recorder::get_time() { return (ros::Time::now() - start_time_).toSec(); }

void Recorder::process() 
{ 
	ros::Rate rate(HZ_);
	while(ros::ok()){
		add_trajectory();
		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"recorder");
	Recorder recorder;
	recorder.process();
	return 0;
}