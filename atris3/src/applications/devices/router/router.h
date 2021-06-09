#ifndef ATRIS_DEVICE_ROUTER_H_
#define ATRIS_DEVICE_ROUTER_H_

#include <string>
#include "router_comm.h"

#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/RobotInfo.h"

class Router {
public:
    ~Router();
    static Router* get_instance()
    {
        static Router singleton;
        return &singleton;
    }
    bool IsMobileWebConnected();
    int GetMobileWebInfo(MobileWebInfo *info);
    int GetMobileWebSignal(int *signal);

private:
    Router();
    void InitSetting();
    int PubRouterInfo();

private:
    IRourer *router_obj_;
    ros::NodeHandle nh_;
    ros::Publisher router_info_pub_;
    static const int kPubSignalSec_ = 10;
};


#endif