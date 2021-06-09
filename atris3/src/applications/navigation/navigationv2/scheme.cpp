#include "scheme.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_sch]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_sch]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_sch]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_sch]"#format, ##args)

namespace nav{

int Scheme::_check_json(Value& scheme)
{
    Value p = scheme["pointInfo"];
    int p_size = p.size();
    if (p_size <= 0) {
        Lerror("P_SIZE <= 0");
        return ERR_INVALID_FIELD;
    }
    for(int i = 0; i < p_size; i++) {
        if (p[i]["pointBaseId"].isNull()) {
            Lerror("pointBaseId null");
        }

        if (p[i]["mapPointId"].isNull()) {
            Lerror("mapPointId null");
        }

        if (p[i]["cameraFoucs"].isNull()) {
            Lerror("cameraFoucs null");
        }
        
        if (p[i]["cameraHangle"].isNull()) {
            Lerror("cameraHangle null");
        }

        if (p[i]["cameraVangle"].isNull()) {
            Lerror("cameraVangle null");
        }

        if (p[i]["cameraZoom"].isNull()) {
            Lerror("cameraZoom null");
        }
        
        if (p[i]["captureInfrared"].isNull()) {
            Lerror("captureInfrared null");
        }

        if (p[i]["captureVisibleLight"].isNull()) {
            Lerror("captureVisibleLight null");
        }

        if (p[i]["heatType"].isNull()) {
            Lerror("heatType null");
        }
        /*
        if (p[i]["locationName"].isNull()) {
            Lerror("locationName null");
        }
        */
        if (p[i]["locationX"].isNull()) {
            Lerror("locationX null");
        }
        if (p[i]["locationY"].isNull()) {
            Lerror("locationY null");
        }
        if (p[i]["locationOrientation"].isNull()) {
            Lerror("locationOrientation null");
        }

        if (p[i]["meterType"].isNull()) {
            Lerror("meterType null");
        }

        if (p[i]["meterModel"].isNull()) {
            Lerror("meterModel null");
        }

        if (p[i]["recognitionType"].isNull()) {
            Lerror("recognitionType null");
        }

        if (p[i]["recordSound"].isNull()) {
            Lerror("recordSound null");
        }
        if (p[i]["recordVideo"].isNull()) {
            Lerror("recordVideo null");
        }

        if (p[i]["saveType"].isNull()) {
            Lerror("saveType null");
        }

        if (p[i]["temperatureFramePoint"].isNull()) {
            Lerror("temperatureFramePoint null");
        }

        if (p[i]["deviceFramePoint"].isNull()) {
            Lerror("deviceFramePoint null");
        }

        //return ERR_INVALID_FIELD;

    }
    return ERR_OK;
}

int Scheme::_task_action(int op){
    Linfo("%s: ", __FUNCTION__);
    return ERR_OK;
}

bool Scheme::_reload(){
    Linfo("%s: ", __FUNCTION__);
    return true;
}

bool Scheme::_construct_task(){
    Linfo("%s: ", __FUNCTION__);
    return true;
}

bool Scheme::_append_action(){
    Linfo("%s: ", __FUNCTION__);
    return true;
}

void Scheme::on_running(){
    Linfo("%s: ", __FUNCTION__);
    if (!_reload()){
        _nxt_st = FIN;
        return;
    }

    if (!_construct_task()){
        _nxt_st = FIN;
        return;
    }

    _append_action();
    Linfo("run task:%s", _tf.name().c_str());
    _thd.push(&_tf);
    _tf.run();
    Linfo("run task:%s ok", _tf.name().c_str());
}

void Scheme::on_finished(){
    Linfo("%s on finished", _name.c_str());
    Notifier::publish_pos_status(STATUS_CODE_9200, _type);
}

void Scheme::on_loop(){
    Linfo("%s on loop", _name.c_str());
    if (_tf.is_finished()){
        Linfo("%s is finished %s", _name.c_str(), _tf.name().c_str());
        _nxt_st = FIN;
    }
}

void Scheme::on_paused(){
    Linfo("%s on paused", _name.c_str());
}

void Scheme::_clear(){
    Linfo("%s clear", _name.c_str());
}

}
