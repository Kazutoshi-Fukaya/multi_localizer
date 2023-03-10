#include "object_localizer/mcl.h"

using namespace object_localizer;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"mcl");
    MCL mcl;
    mcl.process();
    return 0;
}
