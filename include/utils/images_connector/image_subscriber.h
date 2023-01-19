#ifndef IMAGE_SUBSCRIBER_H_
#define IMAGE_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace multi_localizer
{
class ImageSubscriber
{
public:
    ImageSubscriber();
    ImageSubscriber(ros::NodeHandle nh,std::string robot_name,std::string img_topic_name);

    cv::Mat get_img();

private:
    void img_callback(const sensor_msgs::ImageConstPtr& msg);

    // node handle
    ros::NodeHandle nh_;

    // subscriber
    ros::Subscriber img_sub_;

    // buffer
    cv::Mat img_;
};
} // namespace multi_localizer

#endif	// IMAGE_SUBSCRIBER_H_