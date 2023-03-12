#include "multi_localizer/multi_localizer.h"

using namespace multi_localizer;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"multi_localizer");
    MultiLocalizer multi_localizer;
    multi_localizer.process();
    return 0;
}