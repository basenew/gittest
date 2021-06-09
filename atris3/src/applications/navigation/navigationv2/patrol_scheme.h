#pragma once
#include <iostream>
#include <string.h>
#include <json/json.h>
#include "nav_error_code.h"
#include "task_node.h"
#include "task_flow.h"
#include "gs_api.h"
#include "task.h"
#include "navigation.h"
#include "scheme.h"
#include "ros/ros.h"
#include "atris_defines.h"
#include "atris_msgs/PtzControl.h"
#include "atris_msgs/LampCmd.h"
#include "atris_msgs/PPPlayerControl.h"
#include "atris_msgs/GetPPPlayingStatus.h"
#include "atris_msgs/PPPlayerControl.h"
#include "atris_msgs/GetGpsPos.h"
#include "atris_msgs/ChargeCmd.h"
#include "platform/udock/UdockData.h"
#include "platform/pm/PMData.h"
#include "imemory/atris_imemory_api.h"
#include "kv_db.h"
#include "scheduler/scheduler.h"
#include "platform/gs/GsData.h"
#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[sch_pat]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[sch_pat]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[sch_pat]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[sch_pat]"#format, ##args)

using namespace std;
using namespace Json;

namespace nav{
using string_vct= vector<string>;
enum PATROL_MODE{
    PATMD_GO = 0,
    PATMD_GO_AND_COME = 1
};

class Patrol:public Scheme{
public:
    inline Patrol()
    :Scheme("patrol", Task::TASK_PATROL)
    ,_enable(true)
    ,_chg_enable(true)
    ,_chg_pt_name("charge"){
    charge_cmd_pub_ = nh_.advertise<atris_msgs::ChargeCmd>(TOPIC_CHARGE_CMD_MESSAGE, 100);
    ptz_control_srv_client_ = nh_.serviceClient<atris_msgs::PtzControl>(SRV_PTZ_CONTROL);
    signal_msg_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_REQUEST_MESSAGE, 10);
    _db.select("pat");
    };

    inline virtual ~Patrol(){
    };

    inline bool is_charging(){
        shm::ChargeState shm_state;
        shm::iMemory_read_ChargeState(&shm_state);
        Linfo("%s st:%d", __FUNCTION__, shm_state.state);
        return shm_state.state == DOCK_SUCESS;
    };
    
    int plan_route(string schemeName, string taskType, string taskMouldId, std::string taskTimestamp);
/*
    inline bool save_to_db(){
        string map_name = MapManager::get_instance().get_using_map();
        Linfo("%s map:%s sch:%s times:%d interval:%d",
              __FUNCTION__, map_name.c_str(), _sch_name.c_str(), _total_times, _interval);
        if (map_name.empty() || _sch_name.empty()) return false;
        _db.set("map", map_name);
        _db.set("sch", _sch_name);
        _db.set_int("times", _total_times); 
        _db.set_int("interval", _interval);

        return true;
    };

    inline bool rm_from_db(){
        Linfo("%s", __FUNCTION__);
        _db.del("map");
        _db.del("sch");
        _db.del("times"); 
        _db.del("interval");
        return true;
    };

    inline bool have_task_to_run(){
        bool ret = false;
        string using_map = MapManager::get_instance().get_using_map_from_db();
        string map = _db.get("map");
        string sch = _db.get("sch");
        int times = _db.get_int("times");
        int interval = _db.get_int("interval");
        ret = !using_map.empty() && map == using_map && !sch.empty() && times > 0;
        Linfo("%s map:%s using map:%s sch:%s times:%d interval:%d ret:%d",
              __FUNCTION__, map.c_str(), using_map.c_str(), sch.c_str(), times, interval, ret);

        return ret;
    };


    inline int publish_leave_piple(){
        Linfo("%s", __FUNCTION__);
        atris_msgs::ChargeCmd msg;
        msg.cmd = PM_CMD_PC_LEAVE_PILE;
        msg.data = { 0 };
        msg.data[0] = 0x02;
        charge_cmd_pub_.publish(msg);
        return ERR_OK;
    };

    inline int start_by_self(){
        Linfo("is charging start by self");
        int ret = publish_leave_piple();
        if (ret == ERR_OK){
            ret = save_to_db();
            if (ret == ERR_OK){
                Utils::get_instance()->publish_event(EVT_LEAVE_PILE_PATROL, EVT_TRIGGERED);
            }
        }
        return ret;
    };
*/
    inline const GsNavPoint& return_point(){return _ret_pt;};  
    inline void return_point(const GsNavPoint &ret_pt){_ret_pt = ret_pt;};     

    inline const string& charge_point_name(){return _chg_pt_name;};     
    inline void charge_point_name(const string& chg_pt_name){_chg_pt_name = chg_pt_name;};     

    inline bool enable(){return _enable;}; 
    inline void enable(bool enable){_enable = enable;};     

    inline bool charge_enable(){return _chg_enable;}; 
    inline void charge_enable(bool chg_enable){_chg_enable = chg_enable;};

    virtual int parse(const string &scheme_name, Value& scheme);
    void notifyPatrolStatus(int status);

protected:
    virtual void on_running();
    virtual void on_loop();
    virtual void on_finished();

    virtual int  _task_action(int op);
    virtual bool _reload();
    virtual bool _construct_task();
    virtual bool _append_action();
    virtual void _clear();

private:
    int _start_time;
    bool _chg_enable;
    bool _enable;

    string _chg_pt_name;

    KVDB   _db;

    Operation       _sch_op;
    GsNavPoint      _ret_pt;
    list<ObsPath>   _paths;
    list<ObsPoint>  _points;
    list<Task*>     _tasks;
    
    ros::NodeHandle nh_;
    ros::Publisher lamp_cmd_pub_;
    ros::ServiceClient get_ppplaying_status_srv_client_;
    ros::ServiceClient ppplayer_control_srv_client_;
    ros::ServiceClient ptz_control_srv_client_;
    ros::Publisher charge_cmd_pub_;
    ros::Publisher signal_msg_pub_;

    list<Task*>     _ptz_tasks;

    sched::Scheduler scheduler;
};


}

