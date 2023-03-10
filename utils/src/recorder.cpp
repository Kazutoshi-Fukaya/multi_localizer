#include "recorder/recorder.h"

using namespace recorder;

Recorder::Recorder()
{
    std::cerr << "set 'record_file_path'" << std::endl;
}

Recorder::Recorder(std::string record_file_path) :
    record_file_path_(record_file_path)
{
    std::cout << "record_file_path: " << record_file_path_ << std::endl;
}

void Recorder::add_trajectory(double time,Pose est_pose,Pose ref_pose)
{
    Trajectory trajectory;
    trajectory.time = time;
    trajectory.est_pose = est_pose;
    trajectory.ref_pose = ref_pose;
    trajectories_.emplace_back(trajectory);
}

void Recorder:: add_observation(double time,std::string name)
{
    observations_.emplace_back(Observation(time,name));
}

void Recorder::save_csv()
{
    save_trajectories();
    save_observations();
}

void Recorder::save_trajectories()
{
    if(trajectories_.empty()){
        std::cerr << "trajectories are empty" << std::endl;
        return;
    }
    std::string file_name = record_file_path_ + "trajectory_" + get_date() + ".csv";
    static std::ofstream traj_ofs(file_name);
    for(const auto &traj : trajectories_){
        traj_ofs << traj.time << ","
                 << traj.est_pose.x << "," << traj.est_pose.y << "," << traj.est_pose.yaw << ","
                 << traj.ref_pose.x << "," << traj.ref_pose.y << "," << traj.ref_pose.yaw << std::endl;
    }
    traj_ofs.close();
    std::cout << "\nSave: " << file_name << std::endl;
}

void Recorder::save_observations()
{
    if(observations_.empty()){
        std::cerr << "observations are empty"<< std::endl;
        return;
    }
    std::string file_name = record_file_path_ + "observation_" + get_date() + ".csv";
    static std::ofstream obs_ofs(file_name);
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
