#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/utils.h>

#include <sstream>
#include <fstream>

class Sampler
{
public:
	Sampler();
	~Sampler();
	void process();

private:
	class Pose
	{
	public:
		Pose() :
			x(0.0), y(0.0), theta(0.0) {}
		Pose(double _x,double _y,double _theta) :
			x(_x), y(_y), theta(_theta) {}

		double x;
		double y;
		double theta;
	private:
	};

	class Trajectory
	{
	public:
		Trajectory() :
			time(0.0), est_pose(Pose()), ref_pose(Pose()) {}

		void set_est_pose(double x,double y,double theta)
		{
			est_pose.x = x;
			est_pose.y = y;
			est_pose.theta = theta;
		}

		void set_ref_pose(double x,double y,double theta)
		{
			ref_pose.x = x;
			ref_pose.y = y;
			ref_pose.theta = theta;
		}

		double time;
		Pose est_pose;
		Pose ref_pose;

	private:
	};

private:
	void load_file(std::string file_name);
	std::vector<std::string> split(std::string& input,char delimiter);

	// node handle
	ros::NodeHandle private_nh_;


	// buffer
	std::vector<Trajectory> trajectories_;
	double start_time_;
	int count_;

	// params
	int SAMPLE_NUM_;
	std::string FILE_PATH_;
	std::string RECORD_PATH_;
};

Sampler::Sampler() :
	private_nh_("~"),
	start_time_(0.0),
	count_(0)
{
	private_nh_.param("SAMPLE_NUM",SAMPLE_NUM_,{5});
	private_nh_.param("FILE_PATH",FILE_PATH_,{std::string("")});
	std::string file_name = FILE_PATH_ + "single_object_localizer2.csv";
	load_file(file_name);
}

Sampler::~Sampler()
{
	private_nh_.param("RECORD_PATH",RECORD_PATH_,{std::string("")});
	std::string record_file = RECORD_PATH_ + "single_object_localizer2_" + std::to_string(SAMPLE_NUM_) + ".txt";
	std::ofstream ofs(record_file);
	for(const auto &t : trajectories_){
		ofs << t.time << "," 
		    << t.est_pose.x << "," << t.est_pose.y << "," << t.est_pose.theta << ","
			<< t.ref_pose.x << "," << t.ref_pose.y << "," << t.ref_pose.theta << std::endl;
	}
	ofs.close();
}

void Sampler::load_file(std::string file_name)
{
	std::cout << "load: " << file_name << std::endl;
	std::ifstream ifs(file_name);
	std::string line;
	while(std::getline(ifs,line)){
		std::vector<std::string> strvec = split(line,',');
        try{
			double time = static_cast<double>(std::stod(strvec[0]));
			double x_est = static_cast<double>(std::stod(strvec[1]));
			double y_est = static_cast<double>(std::stod(strvec[2]));
			double theta_est = static_cast<double>(std::stod(strvec[3]));
			double x_ref = static_cast<double>(std::stod(strvec[4]));
			double y_ref = static_cast<double>(std::stod(strvec[5]));
			double theta_ref = static_cast<double>(std::stod(strvec[6]));
			
			if(count_ % SAMPLE_NUM_ == 0)
			{
				Trajectory trajectory;
				trajectory.time = time;
				trajectory.set_est_pose(x_est,y_est,theta_est);
				trajectory.set_ref_pose(x_ref,y_ref,theta_ref);
				trajectories_.emplace_back(trajectory);
			}
			count_++;
		}
		catch(const std::invalid_argument& ex){
            ROS_ERROR("Invalid: %s", ex.what());
        }
        catch(const std::out_of_range& ex){
            ROS_ERROR("out of range: %s", ex.what());
        }
	}
	ifs.close();
}

std::vector<std::string> Sampler::split(std::string& input,char delimiter)
{
	std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.emplace_back(field);
    return result;
}

void Sampler::process() { }

int main(int argc,char** argv)
{
	ros::init(argc,argv,"sampler");
	Sampler sampler;
	sampler.process();
	return 0;
}