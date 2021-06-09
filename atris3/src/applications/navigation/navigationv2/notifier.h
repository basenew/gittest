#pragma once
#include <mutex>
#include <iostream>
#include <json/json.h>

#include "gs_api.h"
#include "gs_nav.h"
#include "wsclient/wsclient.h"
#include "nav_error_code.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "atris_msgs/RobotInfo.h"
#include "platform/gs/GsData.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_ntf]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_ntf]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_ntf]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_ntf]"#format, ##args)

using namespace std;
using namespace Json;
using namespace atris_msgs;

enum udock_state {
    begain_dock           = 0 , //正在上桩
    dock_succuss          = 1 , //上桩成功
    dock_fail             = 2 , //上桩失败
    charge_success        = 3 , //充电成功
    charge_fail           = 4 , //充电失败
    begain_leave_pile     = 5 , //正在下桩
    leave_pile_success    = 6 , //下桩成功
    leave_pile_fail       = 7  //下桩失败
};

namespace nav{

class NavStatusHandle{
public:
    virtual void on_pause(){Linfo("%s", __FUNCTION__);};
    virtual void on_stopped(){Linfo("%s", __FUNCTION__);};
    virtual void on_locate_proc(const int code){Linfo("%s", __FUNCTION__);};
    virtual void on_patrol_finished(){Linfo("%s", __FUNCTION__);};
    virtual void on_localization_error(){Linfo("%s", __FUNCTION__);};
    virtual void on_goal_point_not_safe(){Linfo("%s", __FUNCTION__);};
    virtual void on_path_avoiding_obstacle(){Linfo("%s", __FUNCTION__);};
    virtual void on_path_invalid(){Linfo("%s", __FUNCTION__);};
    virtual void on_path_running(Value json){Linfo("%s", __FUNCTION__);};
    virtual void on_path_block(){Linfo("%s", __FUNCTION__);};
    virtual void on_path_unreachable(){Linfo("%s", __FUNCTION__);};
    virtual void on_path_to_start_point(){Linfo("%s", __FUNCTION__);};
    virtual void on_path_finish(){Linfo("%s", __FUNCTION__);};
    virtual void on_nav_navigating(){Linfo("%s", __FUNCTION__);};
    virtual void on_nav_block(){Linfo("%s", __FUNCTION__);};
    virtual void on_nav_planning(){Linfo("%s", __FUNCTION__);};
    virtual void on_nav_unreachable(){Linfo("%s", __FUNCTION__);};
    virtual void on_nav_unreached(){Linfo("%s", __FUNCTION__);};
    virtual void on_nav_reached(){Linfo("%s", __FUNCTION__);};
};

enum StatusCode{
    STATUS_CODE_9000 = 9000, //idle
    STATUS_CODE_9100 = 9100, //execute task start
    STATUS_CODE_9101 = 9101, //execute task end
    STATUS_CODE_9200 = 9200  //end patrol
};

using NavStatusCB = std::function<void(string&)>;

class Notifier{
public:
    inline bool init(){
        Linfo("%s", __FUNCTION__);
        /*
        bool st_connected = connect_ws_status();
        bool health_connected = connect_ws_health();
        return st_connected && health_connected;
        */
         return true;
    };

    bool connect_ws_status();
    bool connect_ws_health();
    inline void set_status_cb(NavStatusCB cb = nullptr){
        unique_lock<recursive_mutex> lock(_mt);
        _st_cb = cb;
    };

    static int publish_event();
    static int publish_pos();
    static int publish_pos_status(int status_code, int task_type);
    static void publish_status(int status_code, int task_type, const std::string& reason);
    static void publish_msg(const Value& root);
    static void publish_recharge_battery(int level);
    static void publish_task_status(std::string mapName, std::string schemeName, std::string taskType,
                                    std::string taskMouldId, std::string taskTimestamp, std::string taskStatus);
    static void publish_task_point_status(PointInfo pinfo);
    static void publish_udock_status(int status);
	static void publish_route_points(std::string schemeName, std::string taskType, std::string taskMouldId,
			                         std::string taskTimestamp, vector<GsNavPoint> &pos_points,
	                                 vector<GsNavPoint> &route_points);
	static void publish_position(const string& msg);

    static Notifier& get_instance(){
        static Notifier ntf;
        return ntf;
    };
    void do_navability_sub_cb(const atris_msgs::NavAbilityMessage& msg);

private:
    Notifier()
    : _st_cb(nullptr) {
        diag_info_pub_ = nh_.advertise<atris_msgs::RobotInfo>(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100);
        signal_resp_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_RESPONSE_MESSAGE, 100);
        navability_sub_ = nh_.subscribe(TOPIC_NAV_ABILITY_RESPONSE_MESSAGE, 100, &Notifier::do_navability_sub_cb, this);
    }


    void _status_cb(const string &data);
    void _health_cb(const string &data);

#if 0
    void _add_monitor();
    void _on_status_action(int code);
    void _on_status(int code, const Value &json);
    void _on_health(int code, const Value &json);

    void _on_pause();
    void _on_stopped();
    void _on_locate_proc(const int code);
    void _on_patrol_finished();
    void _on_localization_error();
    void _on_goal_point_not_safe();
    void _on_path_avoiding_obstacle();
    void _on_path_invalid();
    void _on_path_running(Value json);
    void _on_path_block();
    void _on_path_unreachable();
    void _on_path_to_start_point();
    void _on_path_finish();
    void _on_nav_navigating();
    void _on_nav_block();
    void _on_nav_planning();
    void _on_nav_unreachable();
    void _on_nav_unreached();
    void _on_nav_reached();
#endif

private:
    WsClient    _ws_status;
    WsClient    _ws_health;
    NavStatusCB _st_cb;
    recursive_mutex _mt;

    int lidar_master_state;
    int lidar_slave_state;
    int rgbd_state;
    int gpsstatus;
    int gyro_state;
    int gyro_state_data;
    int odom_delta_spd;
    ros::NodeHandle nh_;
    ros::Publisher diag_info_pub_;
    ros::Publisher signal_resp_pub_;
    ros::Subscriber navability_sub_;
};

}

