#pragma once
#include <iostream>
#include <json/json.h>

#include "gs_api.h"
#include "gs_nav.h"
#include "wsclient/wsclient.h"
#include "charger.h"
#include "nav_error_code.h"

#include "notifier.h"
#include "map_manager.h"
#include "task.h"
#include "task_thread.h"
#include "locate_task.h"
#include "ros/ros.h"
#include "atris_defines.h"
#include "atris_msgs/AisoundTTS.h"
#include "atris_msgs/PtzControl.h"
#include "atris_msgs/GetPtzStatus.h"
#include "atris_msgs/PowerChargeCmd.h"
#include "atris_msgs/GetVisionResult.h"
#include "atris_msgs/GetPtzTemperature.h"
#include "atris_msgs/AntiDrop.h"
#include "platform/gs/GsData.h"
#include <cmath>

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_pnt]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_pnt]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_pnt]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_pnt]"#format, ##args)

#define NAV_FIND_START_POINT 0
#define NAV_FOLLOW_LINE 1
#define STOP_NAV_TIMEOUT (3)
#if 0
#define STATUS_TIMEOUT              (60)
#define STATUS_TASK_RUNNING_TIMEOUT (300)
#define STATUS_RECONNECT_TIMEOUT    (STATUS_TIMEOUT/3)
#endif

using namespace std;
using namespace Json;
using namespace atris_msgs;

enum color {
    RED = 1,
    ORANGE = 2,
    GREEN = 3,
    CYAN = 4,
    BLUE = 5,
    PURPLE = 6,
    WHITE = 7
};

enum light_style {
    static_style   =  0,// 静止
    breathe_style  =  1,// 呼吸
    flashing_style =  2 // 频闪
};

enum Obstacle_Mode {
    NORMAL_STOP = 0,    //不制动停???
    EMERGENCY_STOP = 1, //制动停障
    BYPASS = 2          //绕障
};

enum Obstacle_Timeout_Stratage {
    TASK_STOP = 0, //停止任务
    RE_ROUTE = 1   //重新规划路线
};

namespace nav{

class Navigation;
class Scheme;
//class Patrol;
//class Charge;

class StayTask:public TaskNode{
public:
    inline StayTask(const string& name = "sty_tsk", TaskType type = TASK_NONE, int loop_ms = 1000)
    :TaskNode(name, type, loop_ms)
    ,_duration(0)
    ,_cur_duration(0){};

    inline int  reset(){_cur_duration = 0;return Task::reset();};
    inline int  duration(){return _duration;};
    inline void duration(int duration){if (duration > 0)_duration = duration;};
    void on_loop(){
        if (++_cur_duration >= _duration){
            Linfo("%s stay duration:%d finished front:[%d|%d]", _name.c_str(), _duration, _front_cnt, _fronts.size()); 
            if (!is_paused())//todo impl in task flow
                _nxt_st = FIN;
            else
                Linfo("%s is paused can't finished");
        }
        else{
          //  Linfo("%s cur duration:%d duration:%d front:[%d|%d]", _name.c_str(), _cur_duration, _duration, _front_cnt, _fronts.size()); 
        }
    };
private:
    int _duration;
    int _cur_duration;
};

struct OpPTZ{
    OpPTZ():swh(0), h(0), v(0), z(0), f(0), capv(0), capi(0), recv(0), reci(0),reca(0){};
    int swh;
    float h;
    float v;
    float z;
    float f;
    int capv;
    int capi;
    int capa;
    int recv;
    int reci;
    int reca;
};



#define PTZ_ACTION_TIMEOUT    (60*10)
#define PTZ_MIN_V_ANGLE       (10)

class PTZTask:public TaskNode{
public:
    inline PTZTask(const string& name = "ptz_tsk", TaskType type = TASK_NONE, int loop_ms = 1000)
    :TaskNode(name, type, loop_ms)
    ,_times(PTZ_ACTION_TIMEOUT)
    ,_thd(nullptr)
    ,_is_action(false) {

    if (!nh_.hasParam("/param/ptzFocustime"))
    {
        nh_.setParam("/param/ptzFocustime",5000);
    }
    if (!nh_.hasParam("/param/Tolerance"))
    {
        nh_.setParam("/param/Tolerance",0.1);
    }
    Linfo("%s %s param list works", _name.c_str(), __FUNCTION__); 
      ptz_control_srv_client_ = nh_.serviceClient<atris_msgs::PtzControl>(SRV_PTZ_CONTROL);
      get_ptz_status_srv_client_ = nh_.serviceClient<atris_msgs::GetPtzStatus>(SRV_GET_PTZ_STATUS);
      vision_srv_client_ = nh_.serviceClient<atris_msgs::GetVisionResult>(SRV_GET_VISION_RESULT);
      get_ptz_temp_client_ = nh_.serviceClient<atris_msgs::GetPtzTemperature>(SRV_GET_PTZ_TEMPERATURE);
      _ptz_focustime = 10000.0;
      _ptz_tolerance = 0.1;
    };

    inline ~PTZTask(){_stop_action();}

    inline virtual int reset() {_cur_times = 0;return Task::reset();};
    inline virtual int stop() {
        _stop_action();
        return Task::stop();
    };

    inline void op(const OpPTZ& ptz){_ptz = ptz;};
    inline OpPTZ& op(){return _ptz;};

    void on_running(){
        Linfo("%s %s...", _name.c_str(), __FUNCTION__);
        if (_ptz.swh == 0){
            Linfo("%s %s switch off", _name.c_str(), __FUNCTION__);
            _nxt_st = FIN;
            return;
        }
/*
        atris_msgs::GetPtzStatus status;
          get_ptz_status_srv_client_.call(status);
          if (status.response.status == atris_msgs::GetPtzStatus::Response::PTZ_NOT_LOGIN_STATUS) {
              Lerror("%s %s ptz not login", _name.c_str(), __FUNCTION__);
              _nxt_st = FIN;
              return;
          }
*/
        _is_action = true;
        _thd = new boost::thread(boost::bind(&PTZTask::action, this));
        _thd->detach();

    };

    void on_finished(){
        Linfo("%s %s move def", _name.c_str(), __FUNCTION__);
        _is_action = false;
        /*
        atris_msgs::PtzControl ptzctrl;
        ptzctrl.request.cmd = atris_msgs::PtzControl::Request::PTZ_MOVE_DEFAULT_POINT;
        ptz_control_srv_client_.call(ptzctrl);
        if (!ptzctrl.response.result){
            Lerror("%s %s move to def point fail", _name.c_str(), __FUNCTION__);
            return;
        }
        Linfo("%s %s move def ok", _name.c_str(), __FUNCTION__);
        */
        Notifier::get_instance().publish_task_point_status(_pinfo);
    };

    void on_loop(){
        Linfo("%s %s times:[%d|%d] front:[%d|%d]...", _name.c_str(), __FUNCTION__, _cur_times, _times, _front_cnt, _fronts.size());
        if ((_st == RNG && ++_cur_times >= _times) || (!_is_action)){
            Lerror("ptz action timeout or _is_action false");
            _nxt_st = FIN;
            _stop_action();
        }
    };

    void action() {
        Linfo("%s %s action [h:%lf|v:%lf|z:%lf]", _name.c_str(), __FUNCTION__, _ptz.h, _ptz.v, _ptz.z);
        float v_angle = _ptz.v;
        float h_angle = _ptz.h;
        float zoom    = _ptz.z;
        unsigned int focus   = _ptz.f;
        atris_msgs::PtzControl ptzctrl;

        Linfo("%s %s move1 def", _name.c_str(), __FUNCTION__);
        /*
        ptzctrl.request.cmd = atris_msgs::PtzControl::Request::PTZ_MOVE_DEFAULT_POINT;
        ptz_control_srv_client_.call(ptzctrl);
        Linfo("%s %s move1 def", _name.c_str(), __FUNCTION__);
        */
		do {
        if (!is_active()){
            Linfo("%s %s exit on step1", _name.c_str(), __FUNCTION__);
			break;
        }


        ptzctrl.request.h_angle = h_angle;
        ptzctrl.request.v_angle = v_angle;
        ptzctrl.request.zoom = zoom;
        ptzctrl.request.cmd = atris_msgs::PtzControl::Request::PTZ_MOVE;
        ptz_control_srv_client_.call(ptzctrl);

        if (!ptzctrl.response.result){
            Lerror("%s %s move v_angle fail:%d", _name.c_str(), __FUNCTION__, ptzctrl.response.result);
            break;
        }

        float diff_h = 0.0f;
		float diff_v = 0.0f;
		float diff_z = 0.0f;
        // float diff_f = 0.0f;
#if 1
        if(nh_.getParam("/param/ptzFocustime", _ptz_focustime))
        // if(1)
        {
            if(_ptz_focustime < 1000.0)
            {
                _ptz_focustime = 1000.0;
            }
        }
        else
        {
            _ptz_focustime = 5000.0;
        }
        if(nh_.getParam("/param/ptzTolerance",_ptz_tolerance))
        // if(1)
        {
            if(_ptz_tolerance < 0.01)
            {
                _ptz_tolerance = 0.1;
            }
        }
        else
        {
            _ptz_tolerance = 0.1;
        }
#endif
		atris_msgs::GetPtzStatus status;
        // waitting for ptz rotate
        for (int i = 0; _is_action && i < 100; i++){

          	get_ptz_status_srv_client_.call(status);
          	diff_h = status.response.pan_angle - h_angle;
          	diff_v = status.response.tilt_angle - v_angle;
          	diff_z = status.response.zoom_value - zoom;

            Linfo("%s %s call GetPtzStatus srv at try No.%d h:%.3f v:%.3f z:%.3f statu:%d dh:%.3f dv:%.3f dz:%.3f focustime:%f tolerance:%f" ,_name.c_str(), __FUNCTION__, i,
                    status.response.pan_angle, status.response.tilt_angle, status.response.zoom_value,status.response.status,
					diff_h, diff_v, diff_v,_ptz_focustime,_ptz_tolerance);

          	if ((fabs(diff_h) <= _ptz_tolerance ) &&  (fabs(diff_v) <= _ptz_tolerance) &&  (fabs(diff_z) <= _ptz_tolerance)){
            	// Linfo("+++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            	Linfo("%s %s ptz move h:%.3f v:%.3f z:%.3f dh:%.3f dv:%.3f dz:%.3f ok", _name.c_str(), __FUNCTION__,
					  status.response.pan_angle, status.response.tilt_angle, status.response.zoom_value,
					  diff_h, diff_v, diff_v);
				this_thread::sleep_for(milliseconds((int)_ptz_focustime)); 
                // this_thread::sleep_for(milliseconds(1000)); 
				break;
          	}
            this_thread::sleep_for(milliseconds(200));
		}

        // waitting fot auto focus
#if 0
        int nFocusEqualCnt = 0;
        for (int i = 0; _is_action && i < 20; i++){

          	get_ptz_status_srv_client_.call(status);
            if(5 == nFocusEqualCnt)
            {
                this_thread::sleep_for(milliseconds(1000));
                break;
            }
            if(fabs(focus - status.response.focus_value) < 2) 
            {
                focus = status.response.focus_value;
                nFocusEqualCnt = nFocusEqualCnt + 1;
            }
            else
            {
                focus = _ptz.f;
                nFocusEqualCnt = 0;
            }
            Linfo("%s %s get Focus value at try No.%d prvfocus:%d focus:%d nFocusEqualCnt:%d "_name.c_str(), __FUNCTION__, i,
                    focus, status.response.focus_value , nFocusEqualCnt);
            this_thread::sleep_for(milliseconds(200));
		}
#endif

        //capture picture
        ptzctrl.request.cmd = atris_msgs::PtzControl::Request::PTZ_CAPTURE;
        ptz_control_srv_client_.call(ptzctrl);
        if (_ptz.capv) {
            if (!ptzctrl.response.visible_light_url.empty()) {
                Ldebug("visible light url :%s", ptzctrl.response.visible_light_url.c_str());
                _pinfo.visiblePicUrl = ptzctrl.response.visible_light_url;
            }else{
                Ldebug("visible light url is not gen");
			}
        }

        if (_ptz.capi) {
            if (!ptzctrl.response.infrared_url.empty()) {
                Ldebug("infrared url :%s", ptzctrl.response.infrared_url.c_str());
                _pinfo.thermometryPicUrl = ptzctrl.response.infrared_url;
            }else{
                Ldebug("infrared url is not gen");
			}
        }
     

        if (_ptz.reci || _ptz.recv) {
            Ldebug("record video action...");
            ptzctrl.request.cmd = atris_msgs::PtzControl::Request::PTZ_RECORD_VIDEO_ON;
            ptz_control_srv_client_.call(ptzctrl);

            //TODO config record time.(default: 10s)
            for (int i = 0;_is_action && i < 100; i++)
                this_thread::sleep_for(milliseconds(100));

            ptzctrl.request.cmd = atris_msgs::PtzControl::Request::PTZ_RECORD_VIDEO_OFF;
            ptz_control_srv_client_.call(ptzctrl);
            if (ptzctrl.response.result == 0) {
                 Ldebug("!!!!visible_light_video_url : %s", ptzctrl.response.visible_light_video_url.c_str());
                _pinfo.videoUrl = ptzctrl.response.visible_light_video_url;
                _pinfo.audioUrl = ptzctrl.response.audio_url;
            }else{
				Ldebug("PTZ_RECORD_VIDEO_ON fail:%d", ptzctrl.response.result);
			}
        }

        if(_ptz.reca){
            Ldebug("record sound action...");
            ptzctrl.request.cmd = atris_msgs::PtzControl::Request::PTZ_RECORD_AUDIO_ON;
            ptz_control_srv_client_.call(ptzctrl);
            
            for(auto i=0;_is_action && i<100;i++)
                this_thread::sleep_for(milliseconds(100));

            ptzctrl.request.cmd = atris_msgs::PtzControl::Request::PTZ_RECORD_AUDIO_OFF;
            ptz_control_srv_client_.call(ptzctrl);
            if(ptzctrl.response.result == 0){
                Ldebug("audio url:%s",ptzctrl.response.audio_url.c_str());
                _pinfo.audioUrl = ptzctrl.response.audio_url;
            }else{
				Ldebug("PTZ_RECORD_VIDEO_OFF fail:%d", ptzctrl.response.result);
			}
        }
    
        if(_pinfo.recognitionType == "meter_read") {
            atris_msgs::GetVisionResult getVisionResult;
            Ldebug("meter_model : %s, image size: %d, index: %s", _pinfo.meterModel.c_str(), ptzctrl.response.imagebuf.size(), _pinfo.meterIndex.c_str());
            if (_pinfo.meterModel.find("未") != std::string::npos) {
                log_debug("meterModel not setting");
                break;
            }

            if (!_pinfo.meterModel.empty() && !ptzctrl.response.imagebuf.empty()) {
                getVisionResult.request.model = _pinfo.meterModel;
                getVisionResult.request.index = _pinfo.meterIndex;
                getVisionResult.request.image =  ptzctrl.response.imagebuf;

                vision_srv_client_.call(getVisionResult);
                Ldebug("getVisionResult errcode : %d err msg : %s, meterResult: %s", 
                        getVisionResult.response.error_code, getVisionResult.response.msg.c_str(), getVisionResult.response.result.c_str());
                if (!getVisionResult.response.error_code) {
                    _pinfo.meterResult = getVisionResult.response.result;
                }
            }
        } else if(_pinfo.recognitionType == "infrared_thermometry") {
            atris_msgs::GetPtzTemperature getPtzTemperature;
            std::string start_x, start_y, end_x, end_y;
            log_debug("temperatureFramePoint pram : %s",_pinfo.temperatureFramePoint.c_str());
            Json::Value root;
            Json::Reader read;
            if (_pinfo.temperatureFramePoint.find("未") != std::string::npos) {
                log_debug("temperatureFramePoint not setting");
                break;
            }

            if (read.parse(_pinfo.temperatureFramePoint, root)) {
                if (!root["startX"].isNull() && !root["startY"].isNull() && !root["endX"].isNull() && !root["endY"].isNull()) {
                    if (root["startX"].asFloat()) {
                        getPtzTemperature.request.start_point_x = root["startX"].asFloat();
                    }

                    if (root["startY"].asFloat()) {
                        getPtzTemperature.request.start_point_y = root["startY"].asFloat();
                    }

                    if (root["endX"].asFloat()) {
                        getPtzTemperature.request.end_point_x =  root["endX"].asFloat();
                    }

                    if (root["endY"].asFloat()) {
                        getPtzTemperature.request.end_point_y = root["endY"].asFloat();
                    }
 
                    log_debug("temp pos input: %f, %f, %f, %f",
                            getPtzTemperature.request.start_point_x, getPtzTemperature.request.start_point_y,
                            getPtzTemperature.request.end_point_x, getPtzTemperature.request.end_point_y);
                    
                    get_ptz_temp_client_.call(getPtzTemperature);

                    Ldebug("getPtzTemperature result: %f", getPtzTemperature.response.temperature_max);
                        _pinfo.meterResult = std::to_string(getPtzTemperature.response.temperature_max);  
                }
            }
        }
		}while(0);

        _is_action = false;
        //Task::stop();
}

    inline void Pinfo(const PointInfo& pinfo) {_pinfo = pinfo;}
    inline const PointInfo& Pinfo() {return _pinfo;}
private:
    inline virtual void _stop_action() {
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        {
            TASK_LOCK();
            if (!_is_action){
                Linfo("%s %s already stopped", _name.c_str(), __FUNCTION__);
                return;
            }
            _is_action = false;
        }
        this_thread::sleep_for(chrono::milliseconds(2000));
        if (_thd){
            Linfo("%s %s delete thread", _name.c_str(), __FUNCTION__);
            _thd->interrupt(); 
            _thd->timed_join(boost::posix_time::milliseconds(500));
            //this_thread::sleep_for(chrono::milliseconds(2000));
            delete _thd;
            _thd = nullptr;
            Linfo("%s %s delete thread ok", _name.c_str(), __FUNCTION__);
        }
    };

private:
    OpPTZ _ptz;
    PointInfo _pinfo;
    int   _times;
    int   _cur_times;
    bool  _is_action;
    float _ptz_focustime; //ptz云台聚焦等待时间（ms）
    float _ptz_tolerance; //ptz云台旋转角度（°）
    boost::thread *_thd;
    ros::NodeHandle nh_;
    ros::ServiceClient ptz_control_srv_client_;
    ros::ServiceClient get_ptz_status_srv_client_;
    ros::ServiceClient vision_srv_client_;
    ros::ServiceClient get_ptz_temp_client_;
    
};

class NavPointTask:public TaskNode{
public:
    friend Navigation;
    friend Scheme;

    inline NavPointTask(const string& name = "", TaskType type = TASK_NONE, int loop_ms = 1000)
    :TaskNode(name, type, loop_ms)
    ,_show_status(true)
    ,_auto_nav_enable(false)
    ,_is_start_point(false)
    ,_st_ws_connected(false)
    ,_status_code(-1)
    ,_status_code_last_rnd(-1)
    ,_front_obstacle_check_flag(0)
    ,_front_obstacle_check_flag_last(0)
    ,front_obstacle_check_time_last_(0.0)
    ,front_obstacle_report_time_last_(0.0)
    ,_navigation_type(0)
    ,blocked_start_to_fade(0)
    ,blocked_start_to_count(0)
    ,tts_sound_once_flg_(false)
    ,front_obstacle_notified_(false)
    ,front_obstacle_start_to_count_(0)
    ,front_obstacle_serial_num_("")
    ,_after_obstacle_remove_count(0)
    ,_is_trigged(false)
    ,_task_when_dock(false)
    ,_status_timeout(Config::get_instance()->nav_task_start_timeout) {
        _anti_drop_sub = nh_.subscribe(TOPIC_ANTI_DROP_STATE, 100, &NavPointTask::on_recv_anti_drop, this);
        _aisound_tts_pub = nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);
        _signal_request_pub = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_REQUEST_MESSAGE, 100);
    };
    
    inline int start(){
        Linfo("11111111111111111111111111111111111");
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        int ret = Task::start();
        if (ret != ERR_OK){
            ret = resume();
        }
        return ret;
    };

    inline int stop(){
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        Linfo("stop all gs task by ret:%d", _ret.ret);
        bool is_current_task_finished = false;
        int timeout = 0;
        Linfo("stop all gs task by ret:%d", _ret.ret);
        Notifier::get_instance().set_status_cb(nullptr);
        int ret = GsApi::get_instance()->nav_task_stop_all();
        /*
        if (ret == ERR_OK) {
            while (!is_current_task_finished) {
                ret = GsApi::get_instance()->nav_task_is_finished(is_current_task_finished);
                if (ret == ERR_OK){
                    if (is_current_task_finished){
                        Linfo("stop all gs task ok");
                        break;
                    } else {
                        if(++timeout > STOP_NAV_TIMEOUT) {
                            Lerror("stop current nav task timeout.");
                            break;
                        }
                        sleep(1);
                    }
                } else {
                    Lerror("check current task if finished by ret:%d %s", ret, get_err_msg(ret));
                    break;
                }
            }
            Ldebug("current nav task really finished.");
        } else {
            Lerror("stop all gs task:%d %s", ret, get_err_msg(ret));
        }
        */
        return Task::stop();
    };

    inline int pause(){
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        if (is_running()){
            int ret = GsApi::get_instance()->nav_task_pause();
            if (ret != ERR_OK)
                Lerror("pause gs task:%d %s", ret, get_err_msg(ret));
            else
                return Task::pause();
        }

        return ERR_OK;
    };

    inline int resume(){
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        if (is_paused()){
            int ret = GsApi::get_instance()->nav_task_resume();
            if (ret != ERR_OK)
                Lerror("resume gs task:%d %s", ret, get_err_msg(ret));
            else
                return Task::resume();
        }

        return ERR_OK;
    };

    inline void on_running(){
        Linfo("NNNNNNNNNNNNN---%s %s---NNNNNNNNNNNN", _name.c_str(), __FUNCTION__);
        _st_ws_connected = false;
        _show_status = true;
        _status_timeout = Config::get_instance()->nav_task_start_timeout;
        _status_code = -1;
        
        int ret = _auto_nav_enable ? _auto_nav_to_point():_to_point();
        if (ret != ERR_OK){
            _ret.ret = ret;
            _nxt_st = FIN;
        }
    };

    inline void on_finished(){
        Linfo("NNNNNNNNNNNNN+++%s %s++++NNNNNNNNNNNN", _name.c_str(), __FUNCTION__);
        if (_type == Task::TASK_NAV_TO || _type == Task::TASK_RETURN){
            Linfo("%s to report task finished 9200", _name.c_str(), __FUNCTION__);
            Notifier::publish_pos_status(STATUS_CODE_9200, _type);
            Linfo("%s to report task finished 9200 22222", _name.c_str(), __FUNCTION__);
        }

        if (_type == Task::TASK_RETURN && _task_when_dock)
        {
            Linfo("[NavPointTask::on_finished] leave pile finish, send request_new_task[%s].", _task_data.c_str());
            atris_msgs::SignalMessage req;
            req.title = "request_new_task";
            req.msg = _task_data.c_str();
            _signal_request_pub.publish(req);
            _task_when_dock = false;
        }

		_is_trigged = false;
    };

    inline void save_task_data(const char *data)
    {
        _task_data = data;
        _task_when_dock = true;
    }

    inline virtual bool is_stop_flow(){return Task::is_stop_flow() && _ret.ret != 407;};

    inline bool auto_nav_enable(){return _auto_nav_enable;};
    inline void auto_nav_enable(bool auto_nav_enable){_auto_nav_enable = auto_nav_enable;};

    inline bool is_start_point(){return _is_start_point;};
    inline void is_start_point(bool is_start_point){_is_start_point = is_start_point;};

    inline const string& name(){return _name;};
    inline void name(const string& name){_name = name;};

    inline void point(const GsNavPoint& pt){_pt = pt;};
    inline const GsNavPoint& point(){return _pt;};

/*
    inline void Pinfo(const PointInfo& pinfo) {_pinfo = pinfo;}
    inline const PointInfo& Pinfo() {return _pinfo;}
*/
    inline virtual bool swap(Task *t){
        NavPointTask* nt = dynamic_cast<NavPointTask*>(t);
        if (nt == nullptr){
            Linfo("convert fail");
            return false;
        }
        GsNavPoint pt = nt->_pt;
        Linfo("%s(%lf,%lf) => %s(%lf,%lf)",
              _name.c_str(), _pt.np.pos.x, _pt.np.pos.y,
              nt->_name.c_str(), nt->_pt.np.pos.x, nt->_pt.np.pos.y);
        nt->_pt = _pt;
        _pt = pt;
        return true;
    };

    void on_loop();
    void detect_front_obstacle();
    void on_recv_anti_drop(const atris_msgs::AntiDrop& msg);
    //void tts_front_obstacle();
    void sendNavTtsText(int key);
    void reportFrontObstacleEvent(int status, std::string serial_num);
    void notify_front_obstacle();
    //void reportObstacleEventToPc(int val);
    void nav_emergency_stop(int state);
    void nav_set_light_color(int color);
    void nav_set_light_style(int style);
    void nav_release_light_ctrl();
    static bool _reparse_json(string &data, string &item, Json::Value &root);
protected:
    void on_status(string &data);
    virtual void on_status_code(string &data, task_lock &lock);
    virtual int _to_point();
    virtual int _auto_nav_to_point();
protected:
    GsNavPoint _pt;
   // PointInfo _pinfo;
    int    _status_code;
    int    _status_code_last_rnd;
    int    blocked_start_to_count;
    int    blocked_start_to_fade;
    int    _navigation_type;
    bool   tts_sound_once_flg_;
    int    front_obstacle_start_to_count_;
    bool   front_obstacle_notified_;
    string front_obstacle_serial_num_;
    int    _front_obstacle_check_flag;
    int    _front_obstacle_check_flag_last;
    double front_obstacle_report_time_last_;
    double front_obstacle_check_time_last_;
    int    _status_timeout;
    bool   _show_status;
    bool   _st_ws_connected;
    bool   _auto_nav_enable;
    bool   _is_start_point;
    bool _is_trigged;
    int _after_obstacle_remove_count;
    string _map_id;
    string _path;    
    //string _pt_name;    
    string _status_data;

    string _task_data;
    bool   _task_when_dock;

    ros::NodeHandle nh_;
    ros::Publisher _aisound_tts_pub;
    ros::Subscriber _anti_drop_sub;
    ros::Publisher charge_request_pub_;
    ros::Publisher _signal_request_pub;
};

class NavChgPointTask:public NavPointTask{
public:
    inline NavChgPointTask()
    :NavPointTask("chg_pt", Task::TASK_CHARGE)
    ,_prev_code(-1),
    _is_leaving_pile(false),
    _is_charging(false)
    {
        charge_request_pub_ = nh_.advertise<atris_msgs::PowerChargeCmd>(TOPIC_POWER_CHARGE_CMD_REQ_MESSAGE, 100);
        charge_state_sub_ = nh_.subscribe(TOPIC_POWER_CHARGE_CMD_RESP_MESSAGE, 100, &NavChgPointTask::on_receive_charge_state, this);
    };

protected:
    virtual void on_status_code(string &data, task_lock &lock);
    virtual int _to_point();
private:
    int _prev_code;
    void on_receive_charge_state(const atris_msgs::PowerChargeCmd& msg);
    ros::NodeHandle nh_;
    ros::Subscriber charge_state_sub_;
    ros::Publisher charge_request_pub_;
    bool _is_leaving_pile;
    bool _is_charging;
};

}
