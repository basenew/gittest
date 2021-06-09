#pragma once
#include <cmath>
#include <iostream>
#include <json/json.h>
#include "nav_error_code.h"
//#include "navigation.h"
#include "task_node.h"
#include "task_flow.h"
#include "gs_api.h"
#include "task.h"
#include "navigation.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/PtzControl.h"
#include "atris_msgs/LampCmd.h"
#include "atris_msgs/PPPlayerControl.h"
#include "atris_msgs/GetPPPlayingStatus.h"
#include "atris_msgs/PPPlayerControl.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_sch]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_sch]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_sch]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_sch]"#format, ##args)

using namespace std;
using namespace Json;

namespace nav{
using string_vct= vector<string>;

enum MODE{
    SINGLE = 1,
    LOOP   = 2,
    RANDOM = 3
};

struct OpBrocast{
    int         swh;
    int         md;
    string_vct  lst;
};

struct OpVision{
    string recognitionType;
	string meterIndex;
    string meterType;
    string meterModel;
    string temperatureFramePoint;
    string deviceFramePoint;
};

struct OpLamp{
    int  swh;
    bool white;
    bool rb;
    bool alarm;
};

class BrocastTask:public TaskNode{
public:
    inline BrocastTask(const OpBrocast& bc)
    : TaskNode("BrocastTask")
    , _bc(bc) {
        get_ppplaying_status_srv_client_ = nh_.serviceClient<atris_msgs::GetPPPlayingStatus>(SRV_GET_PPPLAYING_STATUS);
        ppplayer_control_srv_client_ = nh_.serviceClient<atris_msgs::PPPlayerControl>(SRV_PPPLAYER_CONTROL);
    };

    void on_running(){
        Linfo("%s %s...", _name.c_str(), __FUNCTION__);
        atris_msgs::GetPPPlayingStatus status;
        get_ppplaying_status_srv_client_.call(status);
        if(status.response.status > 0) {
            atris_msgs::PPPlayerControl pppctrl;
            pppctrl.request.play_control = atris_msgs::PPPlayerControl::Request::PPPLAYER_STOP;
            ppplayer_control_srv_client_.call(pppctrl);
            if (!pppctrl.response.result) {
                log_warn("%s ppplayer_control_srv_client stop failed.", __FUNCTION__);
            }
        }

        if (_bc.lst.size() > 0) {
            Json::Value root;
            Json::FastWriter jw;
            for (std::size_t i = 0; i < _bc.lst.size(); i++) {
              root.append(_bc.lst[i]);
            }
            atris_msgs::PPPlayerControl pppctrl;
            pppctrl.request.play_list_json = jw.write(root);
            pppctrl.request.play_mode = _bc.md;
            pppctrl.request.play_loop = true;
            pppctrl.request.play_control = atris_msgs::PPPlayerControl::Request::PPPLAYER_PLAY;
            ppplayer_control_srv_client_.call(pppctrl);
            if (!pppctrl.response.result) {
                log_warn("%s ppplayer_control_srv_client play failed.", __FUNCTION__);
            }
        } else {
            log_warn("%s ppplayer list is emtpy.", __FUNCTION__);
        }
    };

    void on_finished(){
        Linfo("%s %s...", _name.c_str(), __FUNCTION__);
        atris_msgs::PPPlayerControl pppctrl;
        pppctrl.request.play_control = atris_msgs::PPPlayerControl::Request::PPPLAYER_STOP;
        ppplayer_control_srv_client_.call(pppctrl);
        if (!pppctrl.response.result) {
            log_warn("%s ppplayer_control_srv_client stop failed.", __FUNCTION__);
        }
    };

    void on_paused(){
        Linfo("%s %s...", _name.c_str(), __FUNCTION__);
    };

    void on_loop(){
        Linfo("%s %s...", _name.c_str(), __FUNCTION__);
    };

    inline const string& name(){return _name;};
    inline void name(const string& name){_name = name;};

private:
    OpBrocast _bc;
    ros::NodeHandle nh_;
    ros::ServiceClient get_ppplaying_status_srv_client_;
    ros::ServiceClient ppplayer_control_srv_client_;
};

class LampTask:public TaskNode{
public:
    inline LampTask(const OpLamp lamp)
    : TaskNode("LampTask"), _lamp(lamp) {
      lamp_cmd_pub_ = nh_.advertise<atris_msgs::LampCmd>(TOPIC_LAMP_CMD_MESSAGE, 100);
    };

    void on_running(){
        Linfo("%s %s...", _name.c_str(), __FUNCTION__);
        atris_msgs::LampCmd msg;
        msg.status = atris_msgs::LampCmd::STATUS_ON;
        if (_lamp.white) {
            msg.type = atris_msgs::LampCmd::LAMP_TYPE_WHITE;
            lamp_cmd_pub_.publish(msg);
        }
        
        if (_lamp.rb) {
            msg.type = atris_msgs::LampCmd::LAMP_TYPE_FLASH;
            lamp_cmd_pub_.publish(msg);
        }
    };

    void on_finished(){
        Linfo("%s %s...", _name.c_str(), __FUNCTION__);
        atris_msgs::LampCmd msg;
        msg.status = atris_msgs::LampCmd::STATUS_OFF;
        if (_lamp.white) {
            msg.type = atris_msgs::LampCmd::LAMP_TYPE_WHITE;
            lamp_cmd_pub_.publish(msg);
        }
        
        if (_lamp.rb) {
            msg.type = atris_msgs::LampCmd::LAMP_TYPE_FLASH;
            lamp_cmd_pub_.publish(msg);
        }
    };

    void on_paused(){
        Linfo("%s %s...", _name.c_str(), __FUNCTION__);
    };

    void on_loop(){
        Linfo("%s %s...", _name.c_str(), __FUNCTION__);
    };

    inline const string& name(){return _name;};
    inline void name(const string& name){_name = name;};

private:
    OpLamp _lamp;
    ros::NodeHandle nh_;
    ros::Publisher lamp_cmd_pub_;
}; 

struct Operation{
    int swh;
    OpPTZ     ptz;
    OpLamp    lamp;
    OpBrocast bc;
    OpVision  vision;
};

struct ObsPoint{
    string id;
    string      name;
    Operation   op;
    GsPos       pt;
    int point_base_id;
};

struct ObsPath{
    bool              charge;
    string            name;
    Operation         op;
    list<ObsPoint>    points;
    ObsPath():charge(false){}
    bool is_cycle(){
        if (points.size() < 2){
            Lerror("%s obs point < 2", name.c_str());
            return false;
        }

        auto start = points.front().pt; 
        auto end = points.back().pt; 
        auto dst = sqrt((start.x - end.x)*(start.x - end.x)+(start.y - end.y)*(start.y - end.y));
        Linfo("%s dst:%f", name.c_str(), dst);
        return dst < 1.0;
    };
};

enum SCHEME_SWITCH
{
    SW_DISABLE = 0,
    SW_ENABLE  = 1,
    SW_PAUSE   = 2
};

class Scheme:public Task{
public:
    Scheme(const string& name, Task::TaskType type)
    :Task(name, type, 1000)
    ,_thd(_name)
    ,_updated(true)
    ,_valid(false)
    {
        _thd.start();
    };

    virtual ~Scheme(){
    };

    inline void sch_switch(int swh){_swh = swh;};
    inline int  sch_switch(){return _swh;};
    inline int  start(){
        Linfo("scheme.h :--------------------------------------------------------");
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        TASK_LOCK();
        int ret = _next_status(STA, lock);
        if (ret != ERR_OK){
            if (is_paused())
                ret = resume();
        }

        Linfo("%s %s ok", _name.c_str(), __FUNCTION__);
        return ret;
    };
    
    inline int  stop(){
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        TASK_LOCK();
        _tf.stop();
        _next_status(STP, lock);
        Linfo("%s %s ok", _name.c_str(), __FUNCTION__);
        return ERR_OK;
    };

    inline int  pause(){
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        TASK_LOCK();
        if (is_running()){
            _tf.pause();
            _next_status(PUS, lock);
        }

        Linfo("%s %s ok", _name.c_str(), __FUNCTION__);
        return ERR_OK;
    };

    inline int  resume(){
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        TASK_LOCK();
        if (is_paused()){
            _tf.resume();
            _next_status(RSM, lock);
        }

        Linfo("%s %s ok", _name.c_str(), __FUNCTION__);
        return ERR_OK;
    };

    inline bool is_valid(){return _valid;};
    inline const string& scheme_name(){return _sch_name;};
    virtual int parse(const string &scheme_name, Value& scheme) = 0;

protected:
    int _check_json(Value& scheme);//todo charge mebay just return point and path

    virtual void on_running();
    virtual void on_finished();
    virtual void on_paused();
    virtual void on_loop();

    virtual int  _task_action(int op);
    virtual bool _reload();
    virtual bool _construct_task();
    virtual bool _append_action();
    virtual void _clear();

    inline bool _need_update(){return _updated;};

protected:

    int          _swh;
    bool         _updated;
    bool         _valid;
    string       _sch_name;

    TaskThread   _thd;
    TaskFlow     _tf;
};


}

