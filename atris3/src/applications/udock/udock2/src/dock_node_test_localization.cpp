#include "udock2/dock_unit_test_localization.h"
using namespace dock;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cruzr_dock_node");
    ros::NodeHandle nh;

    DockUnit dock_unit(nh);
    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}