#include "task_flow.h"

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

namespace nav{

bool TaskFlow::run(function<void()> on_finish){
    Linfo("%s start", _name.c_str());

    if (ERR_OK == Task::start()){
        _on_finish = on_finish;
        return true;
    }
    Linfo("%s start fail", _name.c_str());
    return false;
}

void TaskFlow::on_finished(){
    Linfo("TTTTTTTTTTTTTTT+++%s %s+++TTTTTTTTTTTTTT", _name.c_str(), __FUNCTION__);
    if (_on_finish)
        _on_finish();
    for (TaskNode* t:_tasks)
        Linfo("after run %s front:%d ready:%d last:%d",
              t->name().c_str(), t->fronts().size(), t->is_ready(), t->is_last());
    _thds.insert(_thds.end(), _act_thds.begin(), _act_thds.end());
    _act_thds.clear();
    Linfo("%s on_finished idle thread:%d active thread:%d", _name.c_str(), (int)_thds.size(), (int)_act_thds.size());
}

void TaskFlow::on_running(){
    Linfo("TTTTTTTTTTTTTTT---%s %s---TTTTTTTTTTTTTT", _name.c_str(), __FUNCTION__);
    Linfo("%s %s", _name.c_str(), __FUNCTION__);
    Linfo("%s0 idle thread:%d active thread:%d", _name.c_str(), (int)_thds.size(), (int)_act_thds.size());
    _thds.insert(_thds.end(), _act_thds.begin(), _act_thds.end());
    _act_thds.clear();
    for (auto thd:_thds){
        thd->clear_task(); 
    }

    for (TaskNode* t:_tasks){
        t->reset_order();
        if (t->is_ready()){
            TaskThread* thd;
            if (_thds.empty()){
                stringstream ss;
                ss << "thd_" << _thd_id++; 
                thd = new TaskThread(ss.str());
                //_thds.push_back(thd);
            }
            else{
                thd = _thds.front();
                _thds.pop_front();
            }
            _act_thds.push_back(thd);
            Linfo("%s1 idle thread:%d active thread:%d", _name.c_str(), (int)_thds.size(), (int)_act_thds.size());
            thd->start();
            thd->push(t);
            Linfo("%s %s %s running1 on thread:%s", _name.c_str(), __FUNCTION__, t->name().c_str(), thd->name().c_str());
        }
    }

    Linfo("%s start ok", _name.c_str());
}

void TaskFlow::on_loop(){
    Linfo("%s wait..", _name.c_str());
    if (_act_thds.empty()){
        Linfo("%s all task thread is idle to top task flow", _name.c_str());
        _nxt_st = FIN;
        return;
    }

    Linfo("%s2 idle thread:%d active thread:%d", _name.c_str(), (int)_thds.size(), (int)_act_thds.size());
    bool finished = true;
    bool force_stop = false;
    list<TaskThread*> thds = _act_thds;
    for (TaskThread* thd:thds){
        if (is_finished()){//todo not use
            Linfo("%s stopped by user", _name.c_str());
            break;
        }

        if (thd->is_exited()){
            Linfo("%s already stopped", _name.c_str());
            continue;
        }
        
        TaskNode* t = (TaskNode*)thd->task();
        assert(t);
        if (t->is_idle()){
            Linfo("%s %s is ready", _name.c_str(), t->name().c_str());
            finished = false;
            t->start();
            continue;
        }

        if (!t->is_finished()){
            finished = false;
            continue;
        }

        Linfo("cur task ret:%d", t->result().ret);
        _ret = t->result();
        if (t->is_stop_flow()){
            force_stop = true;
            Linfo("%s force stop:%d", _name.c_str(), _ret.ret);
            break;
        }

        if (t->is_last()){
            Linfo("%s %s is finished and is last task", _name.c_str(), t->name().c_str());
            _thds.push_back(thd);
            _act_thds.remove(thd);
            Linfo("%s3 idle thread:%d active thread:%d", _name.c_str(), (int)_thds.size(), (int)_act_thds.size());
            continue;
        }
        
        bool only_one = true;
        list<TaskNode*> &behinds = t->behinds();    
        for (auto bt:behinds){
            t->rm_behind(bt);
            Linfo("%s behind task's ready:%d state:%d", _name.c_str(), bt->Task::is_ready(), bt->status());
            if (bt->is_ready()){
                Linfo("%s %s is ready to run task flow", _name.c_str(), bt->name().c_str());
                if (only_one){
                    only_one = false;
                    thd->push(bt);
                    Linfo("%s %s running2 on thread:%s", _name.c_str(), bt->name().c_str(), thd->name().c_str());
                }
                else{
                    Linfo("%s4 idle thread:%d active thread:%d",
                          _name.c_str(), (int)_thds.size(), (int)_act_thds.size());
                    if (_thds.empty()){
                        stringstream ss;
                        ss << "thd_" << _thd_id++; 
                        thd = new TaskThread(ss.str());
                        thd->start();
                    }
                    else{
                        thd = _thds.front(); 
                        _thds.pop_front();
                    }
                    _act_thds.push_back(thd);
                    thd->push(bt);
                    Linfo("%s %s running3 on thread:%s", _name.c_str(), bt->name().c_str(), thd->name().c_str());
                    Linfo("%s5 idle thread:%d active thread:%d",
                          _name.c_str(), (int)_thds.size(), (int)_act_thds.size());
                }
                
                int ret = bt->start();
                if (ret != ERR_OK){
                    Linfo("%s %s start err:%d", _name.c_str(), bt->name().c_str(), ret);
                }

                finished = false;
            }
            else{
                Linfo("%s %s is not ready", _name.c_str(), bt->name().c_str());
            }
        }

        if (t->is_last()){
            Task* tmp = (Task*)thd->task();
            if (tmp == t){
                Linfo("%s2 %s is finished and is last task", _name.c_str(), t->name().c_str());
                _thds.push_back(thd);
                _act_thds.remove(thd);
                Linfo("%s6 idle thread:%d active thread:%d", _name.c_str(), (int)_thds.size(), (int)_act_thds.size());
            }
        }
    }


    if (finished){
        _nxt_st = FIN;
        Linfo("%s all task finished!!!!!!!!!!!!!!!!!!", _name.c_str());
    }
    else if (force_stop){
        Linfo("%s force stop all task", _name.c_str());
        for (TaskThread* thd:_act_thds){
            if (!thd->is_exited()){
                Task* t = thd->task();
                assert(t);
                t->stop();
                Linfo("%s force stop task:%s", _name.c_str(), t->name().c_str());
            }
        }
        _nxt_st = FIN;    
    }
}

}

