#pragma once
#include <iostream>

#include "../navigation/nav_error_code.h"
#include "task.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_tmr]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_tmr]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_tmr]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_tmr]"#format, ##args)

using namespace std;


namespace nav{

class Navigation;
class Scheme;
class Timer:public Task{
public:
    Timer():Task("timer"), _timeout(0){}; 
    void on_loop(){
        Linfo("cur timeout:%d", _timeout);
        if (--_timeout == 0){
            _nxt_st = FIN;
            Linfo("timer is timeout");
        }
    };

    int start(){
        if (_timeout > 0){
            _ret.ret = ERR_OK;
            return Task::start();
        }else{
            return ERR_FAIL;
        }
    };
    void timeout(int timeout){_timeout = timeout;};
    int timeout(){return _timeout;};
public:
    int _timeout;
};

}

