#pragma once
#include <list>
#include <thread>
#include <chrono>
#include <iostream>
#include <functional>

#include "task.h"
#include "task_node.h"

#ifndef NONE_PREFIX
#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_thd]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_thd]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_thd]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_thd]"#format, ##args)
#endif

using namespace std;
using namespace std::chrono;

namespace nav{

class TaskThread
{
public:
    inline TaskThread(const string& name = "")
    :_name(name)
    ,_thd(nullptr)
    ,_running(false)
    ,_exited(false)
    ,_t(nullptr)
    {};
    
    inline virtual ~TaskThread(){
        if (_thd)
        {
            delete _thd;
            _thd = nullptr;
        }
    };

    TaskThread(const TaskThread&)=delete;
    void operator=(const TaskThread&)=delete;

    inline const string name(){
        TASK_LOCK();
        return _name;
    };

    inline void name(const string& name){
        TASK_LOCK();
        _name = name;
    };

    bool start();

    bool stop();

    bool push(Task* t);

    inline Task* task(){return _t;};
    
    inline bool is_exited(){return _exited;};

    inline bool clear_task(){
        TASK_LOCK();
        _t = nullptr;
        return ERR_OK;
    };

private:
    inline static void _thread_proc_cb(TaskThread* t){
        if (t) t->_thread_proc();
    };

    void _thread_proc();
    bool _notify(Task* t);

private:
    bool    _running;
    bool    _exited;
    string  _name;
    thread* _thd;

    Task*   _t;

    task_mutex _mt;
    condition_variable_any _cv;
};

}

