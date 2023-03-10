#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageSaver
{
public:
	ImageSaver();
	void process();

private:
	void image_callback(const sensor_msgs::ImageConstPtr& msg);

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::Subscriber img_sub_;

	// buffer
	int count_;

	// param
	std::string FILE_PATH_;
};

ImageSaver::ImageSaver() :
	private_nh_("~"),
	count_(0)
{
	private_nh_.param("FILE_PATH",FILE_PATH_,{std::string("")});

	img_sub_ = nh_.subscribe("img_in",1,&ImageSaver::image_callback,this);
}

void ImageSaver::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& ex){
        ROS_ERROR("Could not convert to color image");
        return;
    }

	if(count_%10 == 0){
		std::string yolo_img_name = "/yolo/image" + std::to_string(count_/10) + ".jpg";
		cv::imwrite(FILE_PATH_ + yolo_img_name,cv_ptr->image);
		count_++;
	}else count_++;
}

void ImageSaver::process() { ros::spin(); }

int main(int argc,char** argv)
{
	ros::init(argc,argv,"image_saver");
	ImageSaver image_saver;
	image_saver.process();
	return 0;
}