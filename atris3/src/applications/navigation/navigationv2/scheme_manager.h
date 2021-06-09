#pragma once
#include <json/json.h>

#include "../navigation/nav_error_code.h"
#include "timer.h"
#include "scheme.h"
#include "patrol_scheme.h"
#include "charge_scheme.h"
#include "ros/ros.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/GetPatrolStatus.h"
#include "scheduler/scheduler.h"
#include "gaussian.h"
#include "utils/utils.h"
#include "atris_msgs/NetworkState.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[sch_mgr]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[sch_mgr]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[sch_mgr]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[sch_mgr]"#format, ##args)

using namespace std;
using namespace Json;
using namespace atris_msgs;

namespace nav{

enum PatrolStatus{
    PATROL_IDLE,
    PATROL_RUNNING,
    PATROL_PAUSED
};

class SchemeManager{
public:
    bool init(){
        Linfo("%s nav v2 enable", __FUNCTION__);
        //_load_scheme();
        _thd.name("sch_mgr");
        _thd.start();
        _batterys = Config::get_instance()->nav_auto_charge_battery;
        auto timer_cb = bind(&SchemeManager::_start_auto_charge, this);
        _timer.set_action(AFT_FIN, timer_cb);
        _timer_thd.start();
        return true;
    };

    int read_json_file(const string& file_path_name, Value &root);
    int write_json_file(const string &file_path_name, const Value &root);
    int download(const SignalMessage &origin, const string &sch_name, const string &url);
    int upload(const SignalMessage &origin, const string &sch_name, Value& resp, string& result);
    int parse(const string &scheme_name, const string &scheme_file);
    int parse_scheme_file(const string &scheme_file, string &scheme_content); 
    Scheme& get_using_scheme(){return _pat_sch;};
    static SchemeManager& get_instance(){
        static SchemeManager singleton;
        return singleton;
    };
    bool doGetPatrolStatus(atris_msgs::GetPatrolStatus::Request& req, atris_msgs::GetPatrolStatus::Response& res);

    inline int get_patrol_status(){
        switch (_pat_sch.status()){
        case Task::PSD:return PATROL_PAUSED;
        case Task::RNG:return PATROL_RUNNING;
        case Task::IDL:
        case Task::FIN:
        default: return PATROL_IDLE;
        }
    };

    inline bool is_running(){
        return _pat_sch.is_active() || _chg_sch.is_active() || _ret_pt_tsk.is_active();
    };

    inline void stop(){
        _pat_sch.stop();
        _chg_sch.stop();
        _ret_pt_tsk.stop();
    };
private:
    SchemeManager()
    :_ret_pt_tsk("ret_tsk", Task::TASK_RETURN)
    ,_scheme_removed(false)
    ,_scheme_changed(true) 
    ,_is_leaving_pile(false)
    ,is_network_connected(false)
    {
      Linfo("%s", __FUNCTION__);
      _signal_req_sub = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &SchemeManager::_handle_msg, this);
      _charge_cmd_sub = nh_.subscribe(TOPIC_CHARGE_CMD_MESSAGE, 100, &SchemeManager::_handle_charge_msg, this);
      _get_patrol_status_srv = nh_.advertiseService(SRV_GET_PATROL_STATUS, &SchemeManager::doGetPatrolStatus, this);
      _charge_request_pub_ = nh_.advertise<atris_msgs::PowerChargeCmd>(TOPIC_POWER_CHARGE_CMD_REQ_MESSAGE, 100);
      _charge_state_sub_ = nh_.subscribe(TOPIC_POWER_CHARGE_CMD_RESP_MESSAGE, 100, &SchemeManager::on_receive_charge_state, this);
      _network_state_sub_ = nh_.subscribe(TOPIC_NETWORK_STATE, 100, &SchemeManager::on_receive_network_state,this);
       _signal_msg_pub = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_REQUEST_MESSAGE, 10);
       shm::ChargeState shm_state;
       shm_state.state = dockSDKState::IDLE;
       shm::iMemory_write_ChargeState(&shm_state);
    }
    
    int _get_scheme(Value &root, Value &scheme, int &scheme_idx, string &map_name, string &scheme_name);
    int _control_task(Task &task, int sw);
    int _del_scheme(Value &req, Value &resp, string &result);
    int _set_scheme(Value &req, Value &resp, string &result);

    int _get_schedule(Value &req, Value &resp, string &result);
    int _get_schedule_list(Value &req, Value &resp, string &result);
    int _sync_schedule(Value &req, Value &resp, string &result);
    int _sync_all_schedule(Value &req, Value &resp, string &result);

    int  _hdl_patrol(const Value& req, Value &resp, string& result);
    int _load_scheme(const string &scheme_name, const string &scheme_file);
    int  _start_auto_charge();

    bool _is_standby(){return _pat_sch.is_valid();};
    void _handle_msg(const atris_msgs::SignalMessage &msg);
    void _handle_charge_msg(const atris_msgs::ChargeCmd &msg);
    void on_receive_charge_state(const atris_msgs::PowerChargeCmd& msg);
    void on_receive_network_state(const atris_msgs::NetworkState& msg);
    string _gen_upload_url(const string &path_name);
    int _hdl_new_task(Value &req, Value &resp, string &result, const SignalMessage &origin);
    int _hdl_remove_task(Value &req, Value &resp, string &result, const SignalMessage &origin);
    int _hdl_return_recharge(Value &req, Value &resp, string &result, const SignalMessage &origin);
    int _hdl_emergency_task(Value &req, Value &resp, string &result, const SignalMessage &origin);
    void _handle_scheme(sched::Event_Instance evi);
    int _schedule_scheme();
    int _parser_cb(string param);
private:
    Patrol _pat_sch;
    Charge _chg_sch;
    Utils *utils;
    sched::Scheduler scheduler;
    std::string _now_time;
    bool         _scheme_removed;
    bool         _scheme_changed;
    bool         _is_leaving_pile;
    TaskThread   _thd;
    TaskThread   _timer_thd;
    Timer        _timer;
    int  _batterys;
    NavPointTask _ret_pt_tsk;
    string       _last_using_map;
    string       _req_param;
    ros::NodeHandle nh_;
    ros::Subscriber _charge_cmd_sub;
    ros::Subscriber _signal_req_sub;
    ros::ServiceServer _get_patrol_status_srv;
    ros::Subscriber _charge_state_sub_;
    ros::Publisher _charge_request_pub_;
    ros::Subscriber _network_state_sub_;
    ros::Publisher _signal_msg_pub;
    bool is_network_connected;

    boost::mutex schedule_mutex;
    boost::mutex scheme_mutex;
};

} 
