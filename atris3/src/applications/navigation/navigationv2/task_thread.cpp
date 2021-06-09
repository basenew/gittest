#include "task_thread.h"

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

namespace nav{

bool TaskThread::start()
{
    Linfo("%s start", _name.c_str());
    TASK_LOCK();
    if (_running) return true;

    _running = true;    
    if (_thd == nullptr)
    {
        _thd = new thread(&_thread_proc_cb, this);
        _thd->detach();
    }

    Linfo("%s start ok", _name.c_str());

    return true;
}

bool TaskThread::stop(){
    Linfo("%s stop", _name.c_str());
    TASK_LOCK();
    if (_thd == nullptr || !_running){
        Linfo("%s already stop", _name.c_str());
        return true;
    }

    if (_t)
        _t->stop();
    _running = false;    
    _cv.notify_one();
    lock.unlock();
    
    if (_thd->joinable())
        _thd->join();

    Linfo("%s stop ok", _name.c_str());
    return true;
}

bool TaskThread::_notify(Task* t){
    if (t == nullptr){
        Lerror("%s push task is null", _name.c_str());
        return false;
    }

    if (_t){
        Linfo("%s push [%s:%d=>%s:%d]",
              _name.c_str(), t->name().c_str(), t->status(), _t->name().c_str(), _t->status());
    }
    else{
        Linfo("%s push %s:%d", _name.c_str(), t->name().c_str(), t->status());
    }

    TASK_LOCK();
    if (_t && _t->is_active()){//todo meby force to stop pre task
        Linfo("%s pre task still running", _name.c_str());
        return false;
    }

    _t = t;
    _t->reset();//todo
    _cv.notify_one();
    return true;
}

bool TaskThread::push(Task* t){
    if (_notify(t)){
        _t->Task::wait_ready();
        Linfo("%s push %s ok", _name.c_str(), _t->name().c_str()); 
        return true;
    }
    Linfo("%s push %s fail", _name.c_str(), _t->name().c_str()); 
    return false;
}

void TaskThread::_thread_proc()
{
    Linfo("%s tid:%d running...", _name.c_str(), this_thread::get_id());
    while (_running)
    {
        {
        Linfo("%s tid:%d wait pushed task...", _name.c_str(), this_thread::get_id());
        TASK_LOCK();
        while (_running && (_t == nullptr || _t->is_finished()))
            _cv.wait_for(lock, milliseconds(10));
    
        Linfo("%s wait:%s st:%d ok", _name.c_str(), _t->name().c_str(), _t->status());

        if (!_running)
        {
            Linfo("%s break...", _name.c_str());
            break;
        }
        }

        if (!_t->is_finished()){
            Linfo("%s tid:%d isn't finished", _t->name().c_str(), this_thread::get_id());
            _t->wait();
            Linfo("%s tid:%d wait finished", _t->name().c_str(), this_thread::get_id());
        }
        else
            Linfo("%s is finished", _t->name().c_str());
    }
    _exited = true;
    Linfo("%s tid:%d exit...", _name.c_str(), this_thread::get_id());
}

}

