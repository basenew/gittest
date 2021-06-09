
#include "scan_trans.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "multi_to_single");
    
    MultiToSingle IP;

    ROS_INFO("\033[1;32m---->\033[0m MultiToSingle Started.");

    ros::spin();
    return 0;
}
