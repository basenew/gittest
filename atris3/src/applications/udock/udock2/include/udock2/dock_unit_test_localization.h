#ifndef _DOCK_UNIT_
#define _DOCK_UNIT_
#include <visualization_msgs/Marker.h>
#include "udock2/dock_recognizer.h"


namespace dock
{
class DockUnit
{
public:
    DockUnit(ros::NodeHandle nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_scan_sub_;
    ros::Publisher marker_pub_;
    visualization_msgs::Marker marker_;
    DockRecognizerLine dock_recognizer_;

    void laser_scan_callback(const sensor_msgs::LaserScanConstPtr& msg_ptr);
};
}


#endif