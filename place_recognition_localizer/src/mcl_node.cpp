#include "place_recognition_localizer/mcl.h"

using namespace place_recognition_localizer;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"place_recognition_localizer");
    MCL mcl;
    mcl.process();
    return 0;
}
