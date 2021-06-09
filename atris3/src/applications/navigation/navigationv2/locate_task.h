#pragma once
#include <iostream>
#include <json/json.h>

#include "gs_api.h"
#include "gs_nav.h"
#include "task.h"
#include "task_thread.h"
#include "wsclient/wsclient.h"
#include "map_manager.h"
#include "../navigation/nav_error_code.h"

#define LOCATOR_TASK  "Locator"
#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_lct]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_lct]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_lct]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_lct]"#format, ##args)

using namespace std;
using namespace Json;
using namespace atris_msgs;


namespace nav{

class Navigation;
class Scheme;
class Locator:public Task{
public:
    Locator():Task(LOCATOR_TASK){loop_ms(1000);}; 

    int is_located();

    inline void on_finished(){
        Linfo("%s %s...", _name.c_str(), __FUNCTION__);
    };

    int stop(){
        if (is_active()){
            GsApi::get_instance()->nav_relocate_stop();    
            Task::stop();
        }    
        return ERR_OK;
    };

    void on_running();
    void on_loop();
    void on_status(string& data);
    void response(int ret, string result = "");

    
    inline static Locator& get_instance(){
        static Locator lct;
        return lct;
    };

public:
    string map;
    string point;
    double    x,y;
    int    times;
    double  angle;

    SignalMessage  org;
    GsLocateType   type; 
private:
    int    _status_code;
};

}

