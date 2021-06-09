#include <thread> 
#include "task.h"

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

using namespace std;
namespace nav{

bool Task::wait_ready(int ms){
    Linfo("%s wait ready try lock...", _name.c_str());
    TASK_LOCK();
    Linfo("%s wait ready locked...", _name.c_str());
    while(!_ready && _st != FIN){
        Linfo("%s wait ready...", _name.c_str());
        if (ms == FOREVER)
          _cv.wait(lock);
        else
          _cv.wait_for(lock, milliseconds(ms));
    }

    Linfo("%s wait ready ok...", _name.c_str());
    return _ready || _st == FIN;
}

void Task::wait(int ms){
    Linfo("%s wait...", _name.c_str());
    int loop_ms = _loop_ms;
    while (true){
        Linfo("%s try lock...", _name.c_str());
        this_thread::sleep_for(chrono::milliseconds(10));
        TASK_LOCK();
        Linfo("%s locked...", _name.c_str());
        if (!_ready){
            if (_st == FIN){
                _cv.notify_one();
                Lwarn("%s already finished", _name.c_str());
                return;
            }
            _ready = true;
            _cv.notify_one();
            Linfo("%s notify ready to run", _name.c_str());
        }

        while (_st == _nxt_st){
            Linfo("%s tid:%d wait loop...", _name.c_str(), this_thread::get_id());
            if (loop_ms == FOREVER){
                Linfo("%s wait forever...", _name.c_str());
                _cv.wait(lock);
                Linfo("%s wait forever ok:[%d=>%d]", _name.c_str(), _st, _nxt_st);
            }
            else{
                _cv.wait_for(lock, milliseconds(loop_ms));
                Linfo("%s tid:%d wait loop ok", _name.c_str(), this_thread::get_id());
                if (_st == _nxt_st && (_st == RNG || _st == PSD)){
                    _cb?_cb->on_loop():on_loop(); 
                }
            }
        }

        Linfo("***********+++%s status changed:[%d=>%d]+++***********", _name.c_str(), _st, _nxt_st);
        _prv_st = _st;
        _st = _nxt_st;
        _cv.notify_all();
        switch(_st){
        case RNG:
            if (_prv_st != PSD){
                _exec_action(PRE_RNG);

                if (_cb)
                    _cb->on_running();
                else
                    on_running();

                _exec_action(AFT_RNG);
            }
            else{
                _exec_action(PRE_RSM);

                if (_cb)
                    _cb->on_resume();
                else
                    on_resume();

                _exec_action(AFT_RSM);
            }
            break;
        case PSD:
            _exec_action(PRE_PSD);

            if (_cb)
                _cb->on_paused();
            else
                on_paused();

            _exec_action(AFT_PSD);
            break;
        case FIN:
            _exec_action(PRE_FIN);

            if (_cb)
                _cb->on_finished();
            else
                on_finished();

            _exec_action(AFT_FIN);
            _ready = false;
            Linfo("%s really finished", _name.c_str());
            //_cv.notify_all();
            return;
        default:
            Linfo("%s error invalid status [%d:%d]", _name.c_str(), _st, _nxt_st);
            assert(false);
            //todo _cv.notify_all();
            return;
        }
        //_cv.notify_all();
    }
}

bool Task::wait(int st, int ms){
    TASK_LOCK();
    if (st == _st) return true;

    if (ms == FOREVER){
        while (st != _st)_cv.wait(lock);
    }
    else{
        _cv.wait_for(lock, milliseconds(ms));
    }
    return st == _st;
}

int Task::_next_status(OP op, task_lock &lock){
    Linfo("%s ready:%d [%d:%d] op:%d",
          _name.c_str(), _ready, _st, _nxt_st, op);
    //TASK_LOCK();
    if (!_ready){
        Lerror("%s not ready to op:%d", _name.c_str(), op);
        return ERR_TASK_NOT_READY;
    }
    else if (_st != _nxt_st){
        Lerror("%s [%d=>%d] op:%d conflict1", _name.c_str(), _st, _nxt_st, op);
        return ERR_TASK_OP_CONFLICT;
    }

    StatusResult &st_ret = st_trans[_st][op];
    if (!st_ret.ret){
        Lerror("%s [%d=>%d] op:%d conflict2", _name.c_str(), _st, _nxt_st, op);
        return ERR_TASK_OP_CONFLICT;
    }
    else if (st_ret.st == _st){
        Linfo("%s ready:%d [%d:%d] op:%d already trans",
              _name.c_str(), _ready, _st, _nxt_st, op);
    }
    else{
        _nxt_st = st_ret.st;
        if (_nxt_st == FIN && _ret.ret == ERR_OK){
            Linfo("%s stop by manual!!!!!!!!!!!!!!!!!", _name.c_str());
            _ret.ret = STOP_BY_MANUAL;
        }
        Linfo("%s [%d=>%d] op:%d wait----------------", _name.c_str(), _st, _nxt_st, op);
        _cv.notify_all();
        Linfo("%s [%d=>%d] op:%d wait2----------------", _name.c_str(), _st, _nxt_st, op);
        int st = _st;
        while (_st != _nxt_st)
            _cv.wait(lock);
        Linfo("%s [%d=>%d] op:%d wait ok++++++++++++++++", _name.c_str(), st, _nxt_st, op);
    }

    return ERR_OK;
}
}

