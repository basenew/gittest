#pragma once
#include <iostream>
#include <json/json.h>

#include "gs_api.h"
#include "gs_nav.h"
#include "task.h"
#include "task_thread.h"
#include "nav_point_task.h"
#include "locate_task.h"
#include "wsclient/wsclient.h"
#include "map_manager.h"
#include "../navigation/nav_error_code.h"
#include "ros/ros.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/RobotInfo.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_nav]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_nav]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_nav]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_nav]"#format, ##args)

#define DEFAULT_GET_POS_INTERVAL (1000)

using namespace std;
using namespace Json;
using namespace atris_msgs;


namespace nav{

class Navigation{//todo :public Task{
public:
    enum NAV_MODE{
        MD_NONE,
        MD_LOCATE,
        MD_NAVIGATE
    };
    bool init(){
        Linfo("%s nav v2 enable", __FUNCTION__);
        //_gps_thd = new thread(bind(&Navigation::_update_gps_thread, this));
        _pos_thd = new thread(bind(&Navigation::_get_pos_thread, this));
        _thd.start();
        return Notifier::get_instance().init();
    };

    inline void stop(){
        _locator.stop();
        _navigator.stop();
    };

    inline bool is_locating(){
        return _locator.is_active();
    };

    inline bool is_navigating(){
        return _navigator.is_active();
    };

    inline int stop_navigate(){
        if (_navigator.is_active())
            return _navigator.stop();
        return ERR_OK;
    };

    inline bool is_idle(){
        return !is_locating() && !is_navigating();//todo 
    };

    inline bool is_running(){
        return !is_idle();
    };

    inline Locator& get_locator(){return _locator;};

    inline NavPointTask& get_navigator(){return _navigator;};

    static Navigation& get_instance(){
        static Navigation nav;
        return nav;
    };

private:
    inline Navigation()
    :_task_type(Task::TASK_NONE)
    ,_navigator("nav_to")    
    ,_thd("nav")
    ,_rt_pos_enable(false)
    ,_get_pos_itv(DEFAULT_GET_POS_INTERVAL) {
        Linfo("%s", __FUNCTION__);
        _diag_info_pub = nh_.advertise<atris_msgs::RobotInfo>(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100);
        _signal_req_sub = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &Navigation::_handle_msg, this);
    };

    void _handle_msg(const atris_msgs::SignalMessage &msg);

    int _get_position(Value &req, Value &resp, string &result);
    int _hdl_to_point(Value &req, Value &resp, string &result, const SignalMessage &msg);
    int _to_point(Value &req, Value &resp, string &result);
    int _hdl_locate(Value &req, Value &resp, string &result, const SignalMessage &msg);
    int _locate(Value &req, Value &resp, string &result, const SignalMessage &msg);
    int _locate_stop(Value &req, Value &resp, string &result, const SignalMessage &msg);

    void _update_gps_thread();
    void _get_pos_thread();
    void _gps_data_publish();
private:
    int    _gps_status;
    double _lati;
    double _long;
    double _alti;
    bool   _rt_pos_enable;
    int    _get_pos_itv;

    TaskThread      _thd;
    Task::TaskType  _task_type;
    Locator         _locator;
    NavPointTask    _navigator;
    recursive_mutex _mt;
    thread         *_gps_thd;
    thread         *_pos_thd;
    ros::NodeHandle nh_;
    ros::Publisher _diag_info_pub;
    ros::Subscriber _signal_req_sub;
};

}

