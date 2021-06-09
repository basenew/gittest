#pragma once
#include <list>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <type_traits>

#include "task.h"
#include "task_node.h"
#include "task_thread.h"
#include "log/log.h"

#ifndef NONE_PREFIX
#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_tsf]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_tsf]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_tsf]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_tsf]"#format, ##args)
#endif

using namespace std;

namespace nav{

class TaskFlow:public Task 
{
public:
    TaskFlow(const string& name = "")
    :Task("tf_"+name, Task::TASK_NONE, 1000), _thd_id(0)
    {
    };

    virtual ~TaskFlow(){
        clear();
    };

    TaskFlow(const TaskFlow&)=delete;
    void operator=(const TaskFlow&)=delete;

    inline void push(TaskNode* t){_tasks.push_back(t);};

    virtual bool run(function<void()> on_finish = nullptr);

    inline virtual int start(){
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        int ret = ERR_OK;
        if (is_paused()){
            Linfo("%s is paused", _name.c_str());
            return resume();
        }
        TASK_LOCK();
        return _next_status(STA, lock);
    };

    inline virtual int stop(){
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        TASK_LOCK();
        int ret = ERR_OK;
        for (auto t:_tasks){
            Linfo("%s stop %s by manual", _name.c_str(), t->name().c_str());
            ret = t->stop();
            if (ret != ERR_OK){//todo force to trans status?
                Linfo("%s stop %s by manual err:%s", _name.c_str(), t->name().c_str(), get_err_msg(ret));
                _ret = t->result();
            }
            Linfo("%s stop %s by manual ok", _name.c_str(), t->name().c_str());
        }
        
        ret = _next_status(STP, lock);
        Linfo("%s %s ok", _name.c_str(), __FUNCTION__);
        return ret;
    };
    
    inline virtual int pause(){
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        TASK_LOCK();
        int ret = ERR_OK;
        if (is_running()){
            for (auto t:_tasks){
                if (t->is_running()){
                    ret = t->pause();
                    break;
                }
            }
            if (ret == ERR_OK){
                ret = _next_status(PUS, lock);
            }
        }
        else
            Linfo("%s isn't running", _name.c_str());
        return ret;
    };

    inline virtual int resume(){
        Linfo("%s %s", _name.c_str(), __FUNCTION__);
        TASK_LOCK();
        int ret = ERR_OK;
        if (is_paused()){
            for (auto t:_tasks){
                if (t->is_paused()){
                    ret = t->resume();
                    break;
                }
            }
            if (ret == ERR_OK){
                ret = _next_status(RSM, lock);
            }
        }
        else
            Linfo("%s isn't paused", _name.c_str());
        return ERR_OK;
    };

    virtual void on_loop();
    virtual void on_running();
    virtual void on_finished();
    virtual bool empty(){return _tasks.empty();};

    int clear(){
        if (is_running()){
            return ERR_FAIL;
        }

        TASK_LOCK();
        for (auto t:_tasks){
            t->clear();
        }

        _tasks.clear();
        return ERR_OK;
    };

    inline virtual bool is_stop_flow(){return Task::is_stop_flow() && _ret.ret != 407;};

    inline TaskNode* operator[](size_t idx){
        if (_tasks.size() > idx) return _tasks[idx]; 
        return nullptr;
    };

    inline TaskNode* begin(){
        if (!_tasks.empty()) return _tasks[0]; 
        return nullptr;
    };

    inline TaskNode* end(){
        if (!_tasks.empty()) return _tasks[_tasks.size()-1]; 
        return nullptr;
    };

    inline size_t size(){return _tasks.size();};

private:
    vector<TaskNode*>  _tasks;    

    list<TaskThread*>  _thds;    
    list<TaskThread*>  _act_thds;    
    function<void()>   _on_finish;
    int                _thd_id;
};

}
