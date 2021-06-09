#pragma once
#include <list>
#include <thread>
#include <mutex>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <condition_variable>

#include "task.h"
#ifndef NONE_PREFIX
#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_tsn]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_tsn]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_tsn]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_tsn]"#format, ##args)
#endif

using namespace std;
using namespace std::chrono;

namespace nav{

class TaskNode:public Task
{
friend class TaskFlow;
friend class TaskThread;

public:
    TaskNode(const string& name = "", TaskType type = TASK_NONE, int loop_ms = FOREVER)
    :Task(name, type, loop_ms)
    ,_front_cnt(0)
    ,_behind_cnt(0){
        Linfo("%s construct", _name.c_str());
    };

    virtual ~TaskNode(){
        Linfo("%s destruct", _name.c_str());
    };

    TaskNode(const TaskNode&) = delete;
    void operator=(const TaskNode&) = delete;

    inline int reset_order(){
        _behind_cnt = _behinds.size();
        _front_cnt  = _fronts.size();
        Linfo("%s reset front:%d behind:%d", _name.c_str(), _front_cnt, _behind_cnt); 
        return ERR_OK;
    };

    inline bool front(TaskNode* t){
        Linfo("%s front %s", _name.c_str(), t->name().c_str());
        auto it = find(begin(_behinds), end(_behinds), t);
        if (it != end(_behinds)) return true;

        ++_behind_cnt;
        ++(t->_front_cnt);
        _behinds.push_back(t);
        t->_fronts.push_back(this);
        return true;
    };
    
    inline bool behind(TaskNode* t){
        Linfo("%s behind %s", _name.c_str(), t->name().c_str());
        auto it = find(begin(_fronts), end(_fronts), t);
        if (it != end(_fronts)) return true;

        Linfo("%s behind %s ok", _name.c_str(), t->name().c_str());
        ++_front_cnt;
        ++(t->_behind_cnt);
        _fronts.push_back(t);
        t->_behinds.push_back(this);
        return true;
    };

    inline bool rm_front(TaskNode* t){
        Linfo("%s rm front %s", _name.c_str(), t->name().c_str());
        auto it = find(begin(_fronts), end(_fronts), t);
        if (it == end(_fronts)) return false;
        
        Linfo("%s rm front %s ok", _name.c_str(), t->name().c_str());
        --_front_cnt;
        --t->_behind_cnt;
        if (t->_behind_cnt < 0) assert(0);
        //todo _fronts.erase(it);
        Linfo("%s front size:%d", _name.c_str(), _front_cnt);
        return true;
    };

    inline bool rm_behind(TaskNode* t){
        Linfo("%s rm behind %s", _name.c_str(), t->name().c_str());
        auto it = find(begin(_behinds), end(_behinds), t);
        if (it == end(_behinds)) return false;

        Linfo("%s rm behind %s ok", _name.c_str(), t->name().c_str());
        --_behind_cnt;
        --t->_front_cnt;
        if (_behind_cnt < 0) assert(0);
        //todo _behinds.erase(it);
        Linfo("%s behind size:%d %s front size:%d", _name.c_str(), _behind_cnt, t->_name.c_str(), t->_front_cnt);
        return true;
    };

    inline bool is_ready(){
        TASK_LOCK();
        Linfo("%s [%d:%d] front size:[%d|%d]", _name.c_str(), _st, _nxt_st, _front_cnt, _fronts.size());
        return _front_cnt == 0;
    };

    inline bool is_last(){
        TASK_LOCK();
        //todo return  _behinds.empty();
        return _behind_cnt == 0; 
    };

    inline list<TaskNode*>& fronts() {return _fronts;};
    inline list<TaskNode*>& behinds(){return _behinds;};

    inline void clear(){_front_cnt = _behind_cnt = 0;_fronts.clear();_behinds.clear();};

protected:
    virtual void on_loop(){
        Linfo("%s on loop", _name.c_str());
        this_thread::sleep_for(seconds(2));
        if (_name == "C"){
            Linfo("%s to stop", _name.c_str());
            _ret.ret = ERR_FAIL;
            _nxt_st = FIN;
        }
    };

    virtual void on_running()
    {
        Linfo("%s on running", _name.c_str());
        if (_name == "C"){
            this_thread::sleep_for(seconds(2));
            Linfo("%s on running finished", _name.c_str());
            Linfo("%s to stop", _name.c_str());
            _ret.ret = ERR_FAIL;
            _nxt_st = FIN;
        }
    };

protected:
    int _front_cnt;
    int _behind_cnt;

    list<TaskNode*> _fronts;
    list<TaskNode*> _behinds;
};

}

