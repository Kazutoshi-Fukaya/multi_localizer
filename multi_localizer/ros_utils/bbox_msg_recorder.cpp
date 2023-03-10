#include <ros/ros.h>

#include <sstream>
#include <fstream>

#include "darknet_ros_msgs/BoundingBoxes.h"

class BboxMsgRecorder
{
public:
	BboxMsgRecorder();
	~BboxMsgRecorder();
	void process();

private:
	class Observation
	{
	public:
		Observation() :
			name(std::string("")), time(0.0) {}
		
		Observation(std::string _name,double _time) :
			name(_name), time(_time) {}

		std::string name;
		double time;
	private:
	};


private:
	void bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);

	// node handle
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// subscriber
	ros::Subscriber bbox_sub_;

	// buffer
	std::vector<Observation> data_;
	ros::Time start_time_;
	bool is_first_;

	// params
	std::string FILE_PATH_;
	int HZ_;
};

BboxMsgRecorder::BboxMsgRecorder() :
	private_nh_("~"),
	is_first_(true)
{
	private_nh_.param("HZ",HZ_,{1});

	bbox_sub_ = nh_.subscribe("bbox_in",1,&BboxMsgRecorder::bbox_callback,this);
}

BboxMsgRecorder::~BboxMsgRecorder()
{
	private_nh_.param("FILE_PATH",FILE_PATH_,{std::string("")});
	std::string file_name = FILE_PATH_ + ".txt";
	std::ofstream ofs(file_name);
	for(const auto &d : data_){
		ofs << d.name << "," << d.time << std::endl;
	}
	ofs.close();
}

void BboxMsgRecorder::bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
	if(msg->bounding_boxes.empty()) return;

	if(is_first_){
		start_time_ = ros::Time::now();
		is_first_ = false;
	}

	double time = (ros::Time::now() - start_time_).toSec();
	for(const auto &bbox : msg->bounding_boxes){
		data_.emplace_back(Observation(bbox.Class,time));
	}
}

void BboxMsgRecorder::process() 
{
	ros::Rate rate(HZ_);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"bbox_msg_recorder");
	BboxMsgRecorder bbox_msg_recorder;
	bbox_msg_recorder.process();
	return 0;
}