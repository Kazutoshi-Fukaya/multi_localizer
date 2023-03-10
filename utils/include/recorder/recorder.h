#ifndef RECORDER_H_
#define RECORDER_H_

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <sstream>

#include "recorder/trajectory.h"
#include "recorder/observation.h"

namespace recorder
{
class Recorder
{
public:
    Recorder();
    Recorder(std::string record_file_path);

    void add_trajectory(double time,Pose est_pose,Pose ref_pose);
    void add_observation(double time,std::string name);
    void save_csv();
    void save_trajectories();
    void save_observations();

private:
    std::string get_date();

    // buffer
    std::vector<Trajectory> trajectories_;
    std::vector<Observation> observations_;
    std::string record_file_path_;
};
} // namespace recorder

#endif  // RECORDER_H_
