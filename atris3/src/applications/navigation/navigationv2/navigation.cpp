#include "map_manager.h"
#include "scheme_manager.h"
#include "navigation.h"
#include "gs_api.h"
#include "database/sqliteengine.h"


#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_nav]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_nav]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_nav]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_nav]"#format, ##args)

#define PARSE_JSON()    if(!reader.parse(msg.msg, req)){\
                            Lerror("parse json fail.");\
                            ret = ERR_INVALID_JSON;\
                            break;\
                        }
namespace nav{


void Navigation::_handle_msg(const atris_msgs::SignalMessage &msg){
    Reader reader;
    Value  req, resp;
    string rsp_title;
    string result;
    int    ret = ERR_OK;

    do
    {
        if (msg.title == "request_robot_pose"){
            PARSE_JSON();
            ret = _get_position(req, resp, result);
        }
        else if(msg.title == "request_locate"){
            PARSE_JSON();
            int sw = req["content"]["switch"].asInt();
            ret = _hdl_locate(req, resp, result, msg);
            if (sw == 1 && ret == ERR_OK)
                return;
        }
        else if(msg.title == "request_nav_to") {
            PARSE_JSON(); 
            if (req["content"]["switch"].isNull()){
                ret = ERR_INVALID_JSON;
                break;
            }
            Linfo("msg content: %s", msg.msg.c_str());
            ret = _hdl_to_point(req, resp, result, msg);
        }
        else if (msg.title == "request_nav_switch_with_shttpd") { 
            PARSE_JSON(); 
            if(!req["content"]["switch"].isNull()){
                int sw = req["content"]["switch"].asInt();
                switch(sw){
                case 0:
                    _navigator.pause();
                    break;
                case 1:
                    _navigator.resume();
                    break;
                case 2:
                    _navigator.stop();
                    break;
                default:
                    ret = ERR_INVALID_VALUE;
                    break;
                }
            } else {
                result = "fail_invalid_data";
                ret = ERR_INVALID_FIELD;
            }
        }
        else if (msg.title == "request_set_speed"){
            PARSE_JSON(); 
            if(!req["content"]["speed"].isNull()) {
                float speed = req["content"]["speed"].asFloat();
                if (speed < 0.3) {
                    GsApi::get_instance()->nav_set_path_speed(0);
                    GsApi::get_instance()->nav_set_nav_speed(0);
                } else if (speed < 0.5) {
                    GsApi::get_instance()->nav_set_path_speed(1);
                    GsApi::get_instance()->nav_set_nav_speed(1);
                } else {
                    GsApi::get_instance()->nav_set_path_speed(2);
                    GsApi::get_instance()->nav_set_nav_speed(2);
                }
            }
            return;
        }
        else if (msg.title == "request_long_and_lati"){
            PARSE_JSON();
            std::stringstream slati; slati << setprecision(20) << _lati;
            std::stringstream slong; slong << setprecision(20) << _long;
            std::stringstream salti; salti << setprecision(20) << _alti;
            resp["state"] = _gps_status;
            resp["lati"] = slati.str();
            resp["long"] = slong.str();
            resp["alti"] = salti.str();
        }
        else if (msg.title == "response_emergency_button_state"){
            Linfo("recv response_emergency_button_state");
            PARSE_JSON(); 
            int st = req["content"]["button"].asInt();
            if (st){
                _locator.stop(); 
                _navigator.pause();
            }
            return;
        }
        else return;
    }while(0);

    if (result.empty())
        result = ret == ERR_OK ? "success":get_err_msg(ret);

    resp["id"] = req["content"]["id"];
    resp["timestamp"] = req["content"]["timestamp"];
    resp["result"] = result;
    resp["result_code"] = ret;
    Utils::get_instance()->responseResult(msg, resp, "response" + msg.title.substr(7));
}

void Navigation::_gps_data_publish()
{
    Json::FastWriter fw;
    Json::Value rbtInfoValue;
    std::string strRbtInfo = "";
    atris_msgs::RobotInfo rbtInfo;
    std::stringstream slati; slati << setprecision(20) << _lati;
    std::stringstream slong; slong << setprecision(20) << _long;
    std::stringstream salti; salti << setprecision(20) << _alti;

    rbtInfoValue["robot_info"]["gps"]["error"] = _gps_status;
    rbtInfoValue["robot_info"]["gps"]["lati"] = slati.str();
    rbtInfoValue["robot_info"]["gps"]["long"] = slong.str();
    rbtInfoValue["robot_info"]["gps"]["alti"] = salti.str();

    rbtInfo.json = fw.write(rbtInfoValue);
    _diag_info_pub.publish(rbtInfo);
}

void Navigation::_update_gps_thread() {
    int status;
    while (true) {
        _gps_status = -1; // gps device unavailable
        if (GsApi::get_instance()->get_gps_data(_lati, _long, _alti, status) == ERR_OK) {
            if (status == 2) {
                _gps_status = 0; // 正常
            } else {
                _gps_status = 1; // 定位精度太差
            }
        }
        _gps_data_publish();
        this_thread::sleep_for(chrono::seconds(3));
    }
}

void Navigation::_get_pos_thread(){
    while (true){
        while (_rt_pos_enable){
            Notifier::publish_pos();
            //Notifier::publish_event();
            this_thread::sleep_for(std::chrono::milliseconds(_get_pos_itv));
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int Navigation::_get_position(Value &req, Value &resp, string& result){
    if (!req["content"]["request_type"].isNull()
    &&  !req["content"]["switch"].isNull()){
        _rt_pos_enable = req["content"]["switch"].asInt() == 1;
        if (_rt_pos_enable && !req["content"]["interval"].isNull()){
            _get_pos_itv = req["content"]["interval"].asInt();
            if (_get_pos_itv <= 0)
                _get_pos_itv = DEFAULT_GET_POS_INTERVAL;
        }

        return ERR_OK;
    }
    else{
        GsPos pos;
        int ret = GsApi::get_instance()->nav_get_pos(pos);
        if (ret == ERR_OK){
            resp["x"] = Value(pos.x);
            resp["y"] = Value(pos.y);
            resp["theta"] = Value(pos.angle);
        } 

        return ret; 
    }
}

int Navigation::_hdl_to_point(Value &req, Value &resp, string& result, const SignalMessage &msg){
    int ret = ERR_OK;
    int sw = req["content"]["switch"].asInt();
    resp["switch"] = req["content"]["switch"];
    switch(sw){
    case 0://todo define enum
        _navigator.stop();
        break;
    case 1:
        if (is_idle()){
            if (is_locating()){
                return ERR_LOCATING;
            } 
            /*
            else if (SchemeManager::get_instance().is_running()){
                return ERR_ON_RUNNING;
            }
            */
#if 0
            ret = Locator::get_instance().is_located(); 
            if (ret != ERR_OK){
                break;
            }
#endif
            GsNavPoint pt;

            if(!req["content"]["mode"].isNull()) {
                pt.mode = req["content"]["mode"].asInt();
            } else {
                pt.mode = -1;
            }

            if (!req["content"]["point_id"].isNull()) {
                pt.id = req["content"]["point_id"].asString();
            } else {
                pt.id = "";
            }

            if (!req["content"]["name"].isNull()) {
               pt.np.name = req["content"]["name"].asString();
            } else {
                pt.np.name = "";
            }

            if (!req["content"]["x"].isNull() && !req["content"]["y"].isNull() 
                    && !req["content"]["angle"].isNull()) {
                pt.np.pos.x = req["content"]["x"].asDouble();
                pt.np.pos.y = req["content"]["y"].asDouble();
                pt.np.pos.angle = req["content"]["angle"].asDouble();
            } else {
                pt.np.pos.x = -1;
                pt.np.pos.y = -1;
                pt.np.pos.angle = -1;
            }

            _navigator._type = Task::TASK_NAV_TO;
            _navigator._pt = pt;

            _thd.push(&_navigator);
            ret = _navigator.start();
        }else if (_navigator.is_paused()){
            ret = _navigator.resume();
        } else {
            ret = ERR_ALREADY_RUNNING;
        }
        break;
    case 2:
        if (is_navigating()){
            ret = _navigator.pause();    
        } else {
            ret = ERR_NOT_RUNNING;
        }
        break;
    default:
        ret = ERR_INVALID_VALUE;
        break;
    }

    return ret;
}

int Navigation::_hdl_locate(Value &req, Value &resp, string& result, const SignalMessage &msg){
    int ret = ERR_OK;
    int sw = req["content"]["switch"].asInt();
    do{
        if (is_navigating() || SchemeManager::get_instance().is_running()){
            ret = ERR_NAVIGATING;
            break;
        }
#if 0
        else if (!is_idle()){
            ret = ERR_ALREADY_RUNNING;
            break;
        }
#endif

        switch (sw){
        case 0:
            _locator.stop();
            break;
        case 1:
            ret = is_idle() ? _locate(req, resp, result, msg):ERR_ALREADY_RUNNING;
            break;
        default:
            break;
        }
    }while (0);

    resp["switch"] = Json::Value(sw);
    return ret;
}

int Navigation::_locate(Value &req, Value &resp, string& result, const SignalMessage &msg){
    //todo thread safe
    //todo judge nav not running
    string using_map = MapManager::get_instance().get_using_map();
    if (using_map.empty()){
        return ERR_MAP_NOT_USING;
    }

    _task_type = Task::TASK_LOCATE;
    _locator.map = using_map;
    _locator.point = req["content"]["name"].asString();
    _locator.angle = req["content"]["angle"].asDouble();
    _locator.x = req["content"]["x"].asDouble();
    _locator.y = req["content"]["y"].asDouble();
    _locator.org = msg;

    Linfo("locate not turn around");
    _locator.type = GS_LOCATE_DIRECT_CUSTMOIZED;

    _thd.push(&_locator);
    return _locator.start();
}

}

