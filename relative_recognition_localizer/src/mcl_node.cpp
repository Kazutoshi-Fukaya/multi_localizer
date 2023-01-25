#include "relative_recognition_localizer/mcl.h"

using namespace relative_recognition_localizer;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"relative_recognition_localizer");
    MCL mcl;
    mcl.process();
    return 0;
}