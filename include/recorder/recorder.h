#ifndef RECORDER_H_
#define RECORDER_H_

#include <ros/ros.h>

#include <fstream>
#include <sstream>

namespace multi_localizer
{
class Recorder
{
public:
    Recorder() :
        private_nh_("~") {}

    Recorder(ros::NodeHandle _private_nh) :
        private_nh_(_private_nh) {}

    void add_trajectory(double time,
                        double est_x,double est_y,double est_yaw,
                        double ref_x,double ref_y,double ref_yaw)
    {
        Trajectory trajectory;
        trajectory.time = time;
        trajectory.est = Pose(est_x,est_y,est_yaw);
        trajectory.ref = Pose(ref_x,ref_y,ref_yaw);
        trajectories_.emplace_back(trajectory);
    }

    void add_observation(double time,std::string name)
    {
        observations_.emplace_back(Observation(time,name));
    }

    void save_csv()
    {
        std::string record_file_path;
        private_nh_.param("RECORD_FILE_PATH",record_file_path,{std::string("")});
        std::string file_name = record_file_path + "trajectory_" + get_date() + ".csv";
        std::ofstream traj_ofs(file_name);
        for(const auto &traj : trajectories_){
            traj_ofs << traj.time << ","
                     << traj.est.x << "," << traj.est.y << "," << traj.est.yaw << ","
                     << traj.ref.x << "," << traj.ref.y << "," << traj.ref.yaw << std::endl;
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

private:
    class Pose
    {
    public:
        Pose() :
            x(0.0), y(0.0), yaw(0.0) {}

        Pose(double _x,double _y,double _yaw) :
            x(_x), y(_y), yaw(_yaw) {}

        double x;
        double y;
        double yaw;

    private:
    };

    class Trajectory
    {
    public:
        double time;
        Pose est;
        Pose ref;

    private:
    };

    class Observation
    {
    public:
        Observation() :
            time(0.0), name(std::string("")) {}

        Observation(double _time,std::string _name) :
            time(_time), name(_name) {}

        double time;
        std::string name;
    private:
    };

private:
    std::string get_date()
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

    // node handler
    ros::NodeHandle private_nh_;

    // buffer
    std::vector<Trajectory> trajectories_;
    std::vector<Observation> observations_;
};
}

#endif  // RECORDER_H_
