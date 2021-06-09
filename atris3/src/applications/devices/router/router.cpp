#include "router.h"

#include <json/json.h>
#include "task_manager/task_manager.h"
#include "config/config.h"
#include "router_gosuncn.h"
#include "router_hongdian.h"


Router::Router()
    : router_obj_(nullptr) {
    router_info_pub_ = nh_.advertise<atris_msgs::RobotInfo>(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100);
    if (Config::get_instance()->router_type == "gosuncn") {
        router_obj_ = new Gosuncn();
    } else if (Config::get_instance()->router_type == "hongdian") {
        router_obj_ = new HongDian();
    } else {
        router_obj_ = new HongDian();
    }

    InitSetting();

    Task task;
    task.cb = boost::bind(&Router::PubRouterInfo, this);
    TaskManager::get_instance()->post_cycle(task, kPubSignalSec_*1000);
}

Router::~Router() 
{
    if (router_obj_) {
        delete router_obj_;
        router_obj_ = nullptr;
    }
}

void Router::InitSetting()
{
    if (router_obj_) {
        router_obj_->InitSetting();
    }
}

int Router::PubRouterInfo()
{
    Json::Value json_value;
    Json::FastWriter writer;
    atris_msgs::RobotInfo router_info;

    bool status = IsMobileWebConnected();
    if (status) {
        json_value["robot_info"]["4g"]["error"] = 0;
    } else {
        json_value["robot_info"]["4g"]["error"] = -1;
    }

    MobileWebInfo info;

    int ret = GetMobileWebInfo(&info);
    if (ret == 0) {
        json_value["robot_info"]["4g"]["level"] = info.signal;
        json_value["robot_info"]["sim"]["iccid"] = info.iccid;
        json_value["robot_info"]["sim"]["imsi"] = info.imsi;
        json_value["robot_info"]["sim"]["cellid"] = info.cellid;
        json_value["robot_info"]["router"]["imei"] = info.imei;
        json_value["robot_info"]["router"]["mac"] = info.mac;
    } else {
        json_value["robot_info"]["4g"]["level"] = 0;
    }

    router_info.json = writer.write(json_value);
    router_info_pub_.publish(router_info);

    return 0;
}

bool Router::IsMobileWebConnected()
{
    bool ret = false;
    if (router_obj_ && router_obj_->IsMobileWebConnected()) {
        ret = true;
    } 

    return ret;
}

int Router::GetMobileWebSignal(int *signal)
{
    if (router_obj_) {
        return router_obj_->GetMobileWebSignal(signal);
    }

    return -1;
}



int Router::GetMobileWebInfo(MobileWebInfo *info)
{
    if (router_obj_) {
        return router_obj_->GetMobileWebInfo(info);
    }

    return -1;
}
