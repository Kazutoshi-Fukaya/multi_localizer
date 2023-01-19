#ifndef IMAGES_CONNECTOR_H_
#define IMAGES_CONNECTOR_H_

#include "utils/images_connector/image_subscriber.h"

namespace multi_localizer
{
class ImagesConnector : public std::vector<ImageSubscriber*>
{
public:
    ImagesConnector();
    void process();

private:
    void init(std::string img_topic_name);
    void publish_img();

    // buffer
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // publisher
    ros::Publisher img_pub_;

    // param
    int HZ_;
};	
} // namespace multi_localizer

#endif	// IMAGES_CONNECTOR_H_