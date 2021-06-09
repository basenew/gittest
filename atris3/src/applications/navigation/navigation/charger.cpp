#include <json/json.h>
#include <boost/thread/thread.hpp>
#include "charger.h"
#include "nav_error_code.h"
#include "database/sqliteengine.h"
#include "task_manager/task_manager.h"
#include "imemory/atris_imemory_api.h"

Charger::Charger() {
    signal_resp_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_RESPONSE_MESSAGE, 100);
    dock_sdk_cmd_pub_ = nh_.advertise<atris_msgs::DockSDKCmd>(TOPIC_DOCK_SDK_CMD, 100);
}

void Charger::init()
{
    gsnav = GsNav::get_instance();
    gsapi = GsApi::get_instance();
}

/**
 * @brief on_recv_reached_charge_point 接收到机器人到达充电点消息
 */
void Charger::on_recv_reached_charge_point()
{
    log_info("on_recv_reached_charge_point");
    atris_msgs::DockSDKCmd dock_sdk_cmd;
    dock_sdk_cmd.cmd = atris_msgs::DockSDKCmd::BEGAIN_DOCK;
    dock_sdk_cmd_pub_.publish(dock_sdk_cmd);

    Task task;
    task.id = -1;
    task.cb = boost::bind(&Charger::del_monitor_task, this);
    task.id = TaskManager::get_instance()->post_delay(task, 500 * 1);
}


int Charger::nav_state_monitor(char *data)
{
    Json::Reader reader;
    Json::Value root;

    if(!reader.parse(data, root)){
        log_error("parse gs state fail.");
        return -1;
    }

    int code = root["statusCode"].asInt();
    //过滤掉相同的状态上报
    if(code == state_code){
        state_count ++;
        if(state_count == 20){
            state_count = 0;
        }
        else
            return 0;
    }
    else{
        state_count = 0;
    }

    state_code = code;
    //不保留检测到障碍物的状态码
    //if(code != 701)
    //state_code = code;
    log_info("nav code:%d", code);

    if(code != 200 || (code >= 800 && code <= 899))
        report_pos_state(state_code);

    if(code == 410){
        on_recv_reached_charge_point();
    }
    else if(code == 411){
        before_code = 411;
    }
    else if(code == 408){
        log_info("before code:%d", before_code);
        if(before_code == 411){
            before_code = 0;
            on_recv_reached_charge_point();
        }
    }

    return 0;
}

void Charger::start_charge()
{
    log_info("Charger::start_charge: on_recv_reached_charge_point");
    atris_msgs::DockSDKCmd dock_sdk_cmd;
    dock_sdk_cmd.cmd = atris_msgs::DockSDKCmd::BEGAIN_DOCK;
    dock_sdk_cmd_pub_.publish(dock_sdk_cmd);

    Task task;
    task.id = -1;
    task.cb = boost::bind(&Charger::del_monitor_task, this);
    task.id = TaskManager::get_instance()->post_delay(task, 500 * 1);
}

void Charger::start_charge(std::string map_name)
{

    log_info("start charge");
    gsnav->del_state_monitor(nav_monitor.id);
    nav_monitor.cb = boost::bind(&Charger::nav_state_monitor, this, _1);
    gsnav->add_state_monitor(nav_monitor);

    gsapi->nav_auto_charge(map_name, "charge");
    before_code = 0;

}

void Charger::stop_charge()
{
    log_info("stop charge");
    gsnav->del_state_monitor(nav_monitor.id);

}

void Charger::report_pos_state(int state_code)
{
    GsPos pos_cur;
    log_info("report pos and state");
    //report state and position
    int ret = gsapi->nav_get_pos(pos_cur);
    if(ret == ERR_OK)
        return;

    Json::Value root;

    Json::FastWriter fw;

    int type = 6;

    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    root["content"]["id"] = uid.str();
    root["title"] = "response_nav_state";
    root["content"]["x"] = Json::Value(pos_cur.x);
    root["content"]["y"] = Json::Value(pos_cur.y);
    root["content"]["navtype"] = Json::Value(type);
    root["content"]["angle"] = Json::Value((float)(pos_cur.angle));
    root["content"]["state"] = Json::Value(state_code);

    std::string req_data = fw.write(root);
    atris_msgs::SignalMessage resp;
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    resp.account = shmrbt.robot.receiver;
    resp.msgID = uid.str();
    resp.msg = req_data;
    signal_resp_pub_.publish(resp);
}

bool Charger::del_monitor_task()
{
    gsnav->del_state_monitor(nav_monitor.id);

    return true;
}
