#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/utils.h>

#include <fstream>
#include <sstream>

namespace multi_localizer
{
class PoseRecorder
{
public:
	PoseRecorder() :
		private_nh_("~"), start_time_(ros::Time::now())
	{
		private_nh_.param("ROBOT_NAME",ROBOT_NAME_,{std::string("")});

		pose_sub_ = nh_.subscribe(ROBOT_NAME_ + "/pose",1,&PoseRecorder::pose_callback,this);
		ref_pose_sub_ = nh_.subscribe(ROBOT_NAME_ + "/amcl_pose",1,&PoseRecorder::ref_pose_callback,this);
	}

	~PoseRecorder()
	{
		private_nh_.param("RECORD_FILE_PATH",RECORD_FILE_PATH_,{std::string("")});
		std::ofstream ofs(RECORD_FILE_PATH_ + get_date() + "_" + ROBOT_NAME_ + ".csv");
        for(const auto&r : record_){
            ofs << r.time << "," 
                << r.est_pose.position.x << ","
                << r.est_pose.position.y << ","
                << tf2::getYaw(r.est_pose.orientation) << ","
                << r.ref_pose.position.x << ","
                << r.ref_pose.position.y << ","
                << tf2::getYaw(r.ref_pose.orientation) << std::endl;
        }
        ofs.close();
	}

	void process() { ros::spin(); }
private:
	void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		Record record;
		record.time = get_time();
		record.est_pose = msg->pose;
		record.ref_pose = ref_pose_.pose;
		record_.emplace_back(record);
	}

	void ref_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
	{
		ref_pose_.header = msg->header;
		ref_pose_.pose = msg->pose.pose;
	}

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

	double get_time() { return (ros::Time::now() - start_time_).toSec(); }

	// node handler
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// subscriber
	ros::Subscriber pose_sub_;
	ros::Subscriber ref_pose_sub_;

	// buffer
	geometry_msgs::PoseStamped ref_pose_;
	ros::Time start_time_;
	struct Record
	{
		double time;
		geometry_msgs::Pose est_pose;
		geometry_msgs::Pose ref_pose;
	};
	std::vector<Record> record_;

	// params
	std::string ROBOT_NAME_;
	std::string RECORD_FILE_PATH_;
};
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"pose_recorder");
	multi_localizer::PoseRecorder pose_recorder;
	pose_recorder.process();
	return 0;
}