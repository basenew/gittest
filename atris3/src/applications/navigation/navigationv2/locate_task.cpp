#include "gs_api.h"
#include "notifier.h"
#include "map_manager.h"
#include "locate_task.h"
#include "database/sqliteengine.h"
#include "imemory/atris_imemory_api.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_lct]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_lct]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_lct]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_lct]"#format, ##args)

#define LOCATE_TIMEOUT  180

namespace nav{

int Locator::is_located(){
    Linfo("%s %s", _name.c_str(), __FUNCTION__);
    string using_map = MapManager::get_instance().get_using_map(); 
    if (using_map.empty()){
        Linfo("%s %s not using map", _name.c_str(), __FUNCTION__);
        return ERR_MAP_NOT_USING;
    }

    string cur_pt, cur_map;
    int ret = GsApi::get_instance()->get_current_init_status(cur_map, cur_pt);
    if (ret == ERR_OK){
//todo
#if 0 
        if (using_map == cur_map){
            ret = GsApi::get_instance()->is_initialize_success()?ERR_OK:ERR_NOT_LOCATED;
            Linfo("%s %s located ret:%d", _name.c_str(), __FUNCTION__, ret);
        }
        else{
            Linfo("%s %s located by other map", _name.c_str(), __FUNCTION__);
            ret = ERR_LOCATED_OTHER_MAP;    
        }
#endif
    }

    return ret;
}

void Locator::on_running(){
    Linfo("%s %s...", _name.c_str(), __FUNCTION__);
    _status_code = 0;
    NavStatusCB cb = std::bind(&Locator::on_status, this, placeholders::_1);
    Notifier::get_instance().set_status_cb(cb);
    times = LOCATE_TIMEOUT;
    if (org.account.empty()) {
        shm::Robot shmrbt;
        shm::iMemory_read_Robot(&shmrbt);
        org.account = shmrbt.robot.receiver;
    }

    int ret = GsApi::get_instance()->nav_relocate(x, y, angle, type, map, point);
    if(ret != ERR_OK){
        Linfo("%s locate err:%d to stop", _name.c_str(), ret);
        response(ret);
        _nxt_st = FIN;
    }
    Linfo("%s %s ok...", _name.c_str(), __FUNCTION__);
}

void Locator::response(int ret, string result){
    //todo if (result.empty()) result = get_err_msg(ret);

    Value resp;    
    resp["id"] = org.msgID;
    resp["state"] = Json::Value(_status_code);
    resp["result"] = ret == ERR_OK?"success":"locate_fail";
    resp["switch"] = Json::Value(1);
    resp["timestamp"] = org.timestamp;
    resp["result_code"] = ret;
    Utils::get_instance()->responseResult(org, resp, "response_locate");
}

#if 0//todo
int Locator::is_located(){
    int ret = gsapi->get_device_status(status);
    if (ret != ERR_OK){ 
        return ret;
    }

    if(!status["locationStatus"].isNull()){
        if (status["locationStatus"].asBool())
            return ERR_OK;
    }

    return ERR_LOCATE_FAIL;
}
#endif

void Locator::on_loop(){
    if (times > 0){
        --times;
        int ret;
        if (GsApi::get_instance()->nav_check_relocate_state(ret) != ERR_OK){
            Lerror("check relocate status err:%d", ret);
            return;
        }

        if(ret == 1){
            Linfo("locate finished ret:%d...", ret);
            Notifier::get_instance().set_status_cb(nullptr);
            GsApi::get_instance()->nav_task_stop_all();

            int ret = is_located();
            response(ret);
            _nxt_st = FIN;
        }
        else if(ret == 0){
            Linfo("on locating times:%d....", times);
        }
        else{
            Lerror("locate fail");
            response(ERR_LOCATE_FAIL, "locate_fail");
            _nxt_st = FIN;
        }
    }
    else {
        Lerror("locate over times");
        GsApi::get_instance()->nav_relocate_stop();
        response(ERR_LOCATE_FAIL, "locate_fail");
        _nxt_st = FIN;
    }
}

void Locator::on_status(string& data){
    Json::Reader reader;
    Json::Value root;

    if(!reader.parse(data.c_str(), root)){
        Lerror("parse gs status js data fail js:%s", data.c_str());
        return;
    }

    int code = root["statusCode"].asInt();
    if(code >= 600 && code <= 603){
        _status_code = code;
    }
}

}
