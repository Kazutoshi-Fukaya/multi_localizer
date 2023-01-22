#include "ros_utils/images_connector/images_connector.h"

using namespace multi_localizer;

ImagesConnector::ImagesConnector() :
    private_nh_("~")
{
    private_nh_.param("HZ",HZ_,{10});

    std::string img_topic_name;
    private_nh_.param("IMG_TOPIC_NAME",img_topic_name,{std::string("detected_image")});
    init(img_topic_name);

    img_pub_ = nh_.advertise<sensor_msgs::Image>("image",1);
}

void ImagesConnector::init(std::string img_topic_name)
{
    this->clear();
    std::string robot_element_list_name;
    private_nh_.param("ROBOT_ELEMENT_LIST",robot_element_list_name,{std::string("robot_element_list")});
    XmlRpc::XmlRpcValue robot_element_list;
    if(!private_nh_.getParam(robot_element_list_name.c_str(),robot_element_list)){
        ROS_ERROR("Cloud not load %s", robot_element_list_name.c_str());
        return;
    }

    ROS_ASSERT(robot_element_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    this->resize(robot_element_list.size());
    for(int i = 0; i < (int)robot_element_list.size(); i++){
        if(!robot_element_list[i]["robot_name"].valid() ||
           !robot_element_list[i]["color"].valid()){
            ROS_ERROR("%s is valid", robot_element_list_name.c_str());
            return;
        }
        if(robot_element_list[i]["robot_name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
           robot_element_list[i]["color"].getType() == XmlRpc::XmlRpcValue::TypeString){
            std::string robot_name = static_cast<std::string>(robot_element_list[i]["robot_name"]);
            std::string color = static_cast<std::string>(robot_element_list[i]["color"]);
            this->at(i) = new ImageSubscriber(nh_,robot_name,img_topic_name);
        }
    }
}

void ImagesConnector::publish_img()
{
    if(this->at(0)->get_img().empty()) return;

    int rows = this->at(0)->get_img().rows;
    int cols = this->at(0)->get_img().cols;

    cv::Mat space(cv::Mat::zeros(rows,50,CV_8UC3));
    cv::Mat base(rows,cols*this->size() + space.cols*((int)this->size() - 1),CV_8UC3);
    for(size_t i = 0; i < this->size(); i++){
        cv::Mat img = this->at(i)->get_img();
        if(img.empty()){
            img = cv::Mat::zeros(rows,cols,CV_8UC3);
        }
        cv::Mat roi_1(base,cv::Rect(i*(img.cols + space.cols),0,img.cols,img.rows));
        img.copyTo(roi_1);

        if((int)i != (int)this->size() - 1){
            cv::Mat roi_2(base,cv::Rect((i + 1)*img.cols + i*space.cols,0,space.cols,space.rows));
            space.copyTo(roi_2);
        }
    }

    /*
    cv::Mat col_space(cv::Mat::zeros(rows,50,CV_8UC3));
    cv::Mat row_space(cv::Mat::zeros(30,cols*(int)(this->size()/2) + col_space.cols*(int)(this->size()/2 - 1),CV_8UC3));
    cv::Mat base(rows*(int)(this->size()/3) + row_space.rows,row_space.cols,CV_8UC3);

    // std::cout << "rows: " << rows << std::endl;
    // std::cout << "cols: " << cols << std::endl;
    // std::cout << "col_space rows: " << col_space.rows << std::endl;
    // std::cout << "col_space cols: " << col_space.cols << std::endl;
    // std::cout << "row_space rows: " << row_space.rows << std::endl;
    // std::cout << "row_space cols: " << row_space.cols << std::endl;
    // std::cout << "base_space rows: " << base.rows << std::endl;
    // std::cout << "base_space cols: " << base.cols << std::endl;

    for(size_t i = 0; i < this->size(); i++){
        cv::Mat img = this->at(i)->get_img();
        if(img.empty()){
            img = cv::Mat::zeros(rows,cols,CV_8UC3);
        }

        if(i <= 2){
            cv::Mat roi_1(base,cv::Rect(i*(img.cols + col_space.cols),0,img.cols,img.rows));
            img.copyTo(roi_1);
            if((int)i != 2){
                cv::Mat roi_2(base,cv::Rect((i + 1)*img.cols + i*col_space.cols,0,col_space.cols,col_space.rows));
                col_space.copyTo(roi_2);
            }
        }
        else{
            if(i == 3){
                cv::Mat roi_3(base,cv::Rect(0,rows,row_space.cols,row_space.rows));
                row_space.copyTo(roi_3);
            }
            cv::Mat roi_4(base,cv::Rect((i - 3)*(img.cols + col_space.cols),rows + row_space.rows,img.cols,img.rows));
            img.copyTo(roi_4);
            if((int)i != 5){
                cv::Mat roi_5(base,cv::Rect((i - 2)*img.cols + (i - 3)*col_space.cols,rows + row_space.rows,col_space.cols,col_space.rows));
                col_space.copyTo(roi_5);
            }
        }
    }

    for(size_t i = 0; i < this->size(); i++){
        std::string str = std::string("roomba") + std::to_string(i + 1);
        cv::Point point;
        if(i < 3) point = cv::Point(i*680 + 450,40);
        else point = cv::Point((i - 3)*680 + 450,560);


        cv::putText(base,str,point,cv::FONT_HERSHEY_SIMPLEX,1.2,cv::Scalar(0,0,0),2);
    }
    */

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",base).toImageMsg();
    img_pub_.publish(img_msg);
}

void ImagesConnector::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        publish_img();
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"image_connector");
    ImagesConnector image_connector;
    image_connector.process();
    return 0;
}
