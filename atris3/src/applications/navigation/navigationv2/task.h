#pragma once
#include <iostream>
#include <atomic>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <assert.h>
#include "ros/ros.h"
#include "log/log.h"
#include "nav_error_code.h"

#define FOREVER         (-1)
#define STOP_BY_MANUAL  (-1)

#ifndef NONE_PREFIX
#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_tsk]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_tsk]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_tsk]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_tsk]"#format, ##args)
#endif

#define TASK_LOCK() unique_lock<task_mutex> lock(_mt) 

using namespace std;
using namespace std::chrono;

namespace nav{
//using task_mutex = mutex;
using task_mutex = recursive_mutex;
using task_lock  = unique_lock<task_mutex>;

struct TaskCB{
    virtual ~TaskCB(){};
    virtual void on_loop()=0;
    virtual void on_running()=0;
    virtual void on_paused()=0;
    virtual void on_resume()=0;
    virtual void on_finished()=0;
};

using task_action = function<int()>;

enum ACTION_TYPE{
    PRE_RNG,
    AFT_RNG,
    PRE_PSD,
    AFT_PSD,
    PRE_RSM,
    AFT_RSM,
    PRE_FIN,
    AFT_FIN,
    ACT_MAX
};

struct TaskAction{
};

struct TaskRet{
    int ret;
    int len;
    void* param;
    TaskRet():ret(ERR_OK), len(0), param(nullptr){};
};

class Task{
public:
    enum TaskType{
        TASK_NONE = 0,
        TASK_PATROL,         //巡逻任务
        TASK_NAV_TO,         //点导航
        TASK_RETURN,         //返回任务
        TASK_LOW_BAT_RETURN, //低电量返回
        TASK_PATROL_AUTO,    //自动巡逻
        TASK_CHARGE,         //回充
        TASK_LOCATE,
        TASK_RETURN_CHARGER_DOOR
    };

    enum STATE
    {
        IDL,   // idle
        RNG,   // running
        PSD,   // paused
        FIN,   // finnished
        ST_MAX // state_max
    };

    enum OP
    {
        STA,  //start
        STP,  //stop
        PUS,  //pause
        RSM,  //resume
        OP_MAX,  //operation_max
    };

    struct StatusResult{
        STATE st;
        bool  ret;
    };

    StatusResult st_trans[ST_MAX][OP_MAX]{
       //   STA         STP         PUS         RSM
        {{RNG,true}, {FIN,true}, {IDL,false}, {IDL,false}},//IDL
        {{RNG,true}, {FIN,true}, {PSD,true }, {RNG,true }},//RNG
        {{RNG,false},{FIN,true}, {PSD,true }, {RNG,true }},//PSD
        {{FIN,false},{FIN,true}, {FIN,false}, {FIN,false}},//FIN
    };

    Task(const string& name = "", TaskType type = TASK_NONE, int loop_ms = 1000)
    :_type(type)
    ,_ready(false)
    ,_cb(nullptr)
    ,_st(IDL)
    ,_prv_st(IDL)
    ,_nxt_st(IDL)
    ,_name(name)
    ,_loop_ms(loop_ms)
    ,_actions(nullptr)
    {
    };

    virtual ~Task(){};

    //todo thread self
    inline int  status(){return _st;};
    inline int  next_status(){return _nxt_st;};
    inline int  loop_ms(){return _loop_ms;};    
    inline void loop_ms(int loop_ms){_loop_ms = loop_ms;};
    inline void type(TaskType type){_type = type;};
    inline void name(const string& name){_name = name;};
    inline const string& name(){return _name;};    
    inline TaskType type(){return _type;};
    inline TaskRet& result(){return _ret;};
    inline task_mutex& mutex(){return _mt;};

    inline virtual bool swap(Task *t){return true;};

    inline void set_action(ACTION_TYPE type, task_action action){
        if (action == nullptr || type < 0 || type >= ACT_MAX)
            return;

        if (_actions == nullptr)
            _actions = new task_action[ACT_MAX]();
        _actions[type] = action;
    };

    inline virtual int start() {
        Linfo("%s start",  _name.c_str());
        TASK_LOCK();
        return _next_status(STA, lock);
    };
    inline virtual int stop()  {
        Linfo("%s stop",   _name.c_str());
        TASK_LOCK();
        return _next_status(STP, lock);
    };
    inline virtual int pause() {
        Linfo("%s pause",  _name.c_str());
        TASK_LOCK();
        return _next_status(PUS, lock);
    };
    inline virtual int resume(){
        Linfo("%s resume", _name.c_str());
        TASK_LOCK();
        return _next_status(RSM, lock);
    };

    inline virtual int reset() {
        Linfo("%s reset", _name.c_str());
        TASK_LOCK();
        //todo if (_st != FIN)
        //    return ERR_FAIL;

        _ready = false;
        _st = _nxt_st = IDL;
        _ret = {};
        return ERR_OK;
    };

    inline virtual bool is_idle()    {TASK_LOCK();return _st == IDL;};
    inline virtual bool is_ready()   {TASK_LOCK();return _ready;    };
    inline virtual bool is_paused()  {TASK_LOCK();return _st == PSD;};
    inline virtual bool is_running() {TASK_LOCK();return _st == RNG;};
    inline virtual bool is_finished(){TASK_LOCK();return _st == FIN;};
    inline virtual bool is_active()  {
        TASK_LOCK();
        return _ready && (_st == PSD || _st == RNG);
    };

    inline virtual bool is_stop_flow(){
        //bool to_stop = ((_st == FIN || _nxt_st == FIN) && _ret.ret > ERR_OK);
        Linfo("%s %s st:%d ret:%d", _name.c_str(), __FUNCTION__, _st, _ret.ret);
        bool to_stop = (_st == FIN && _ret.ret != ERR_OK && _ret.ret != STOP_BY_MANUAL);
        if (to_stop)
            Lerror("%s to stop flow ret:%d", _name.c_str(), _ret.ret);

        return to_stop;
    };

    inline virtual bool wait_running (int ms = FOREVER){return wait(RNG, ms);};
    inline virtual bool wait_paused  (int ms = FOREVER){return wait(PSD, ms);};
    inline virtual bool wait_finished(int ms = FOREVER){return wait(FIN, ms);};

    bool wait_ready(int ms = FOREVER);
    void wait(int ms = FOREVER);
    virtual bool wait(int st, int ms);

protected:
    inline virtual void _exec_action(ACTION_TYPE type){
        if (_actions && type < ACT_MAX){
            task_action action = _actions[type];
            if (action != nullptr && !is_stop_flow() && _ret.ret != STOP_BY_MANUAL){
                Linfo("%s exec action:%d", _name.c_str(), type);
                action();
            }
        }
        else
            Linfo("%s not exec action", _name.c_str());
    };
    inline virtual void on_loop()    {/*Linfo("%s on_loop",  _name.c_str());*/};
    inline virtual void on_running() {Linfo("%s on_running",  _name.c_str());};
    inline virtual void on_paused()  {Linfo("%s on_paused",   _name.c_str());};
    inline virtual void on_resume()  {Linfo("%s on_resume",   _name.c_str());};
    inline virtual void on_finished(){Linfo("%s on_finished", _name.c_str());};

    int _next_status(OP op, task_lock &lock);

protected:    
    int  _st;
    int  _prv_st;
    int  _nxt_st;
    int  _loop_ms;
    bool _ready;

    string         _name;
    TaskRet        _ret;
    TaskCB*        _cb;
    TaskType       _type;
    task_action   *_actions;

    task_mutex  _mt;
    condition_variable_any _cv;
};

}



