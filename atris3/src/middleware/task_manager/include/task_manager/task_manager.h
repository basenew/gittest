#ifndef __TASK_MANAGER_H__
#define __TASK_MANAGER_H__

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <list>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>

typedef boost::function<int ()> task_cb;

typedef enum
{
    TT_ONCE,
    TT_REPEAT
}TaskType;

class Task
{
    public:
        int id;
        int type;
        int64_t realtime;//毫秒
        int64_t cycle_time;//微妙
        task_cb cb;

        Task(){

        };

        Task(const Task & t){
            id = t.id;
            type = t.type;
            realtime = t.realtime;
            cycle_time = t.cycle_time;
            cb = t.cb;
        };

        Task(int type, int64_t time, task_cb cb){
            this->type = type;
            this->realtime = time;
            this->cb = cb;
        };

        bool operator < (const Task &t1){
            return t1.realtime > realtime;
        };
};

class TaskManager
{
    private:
        TaskManager();
        std::list<Task> task_list;
        int task_id;
        pthread_mutex_t mutex;
        pthread_cond_t new_task_cond;
        pthread_cond_t delay_cond;
        pthread_condattr_t cattr;

        void task_exec();
        void show_task();
    public:
        int init();
        int post(Task &task);
        int post_delay(Task &task, int delay_ms);
        int post_cycle(Task &task, unsigned cycle_time);
        int del(int task_id);

        static TaskManager* get_instance(){
            static TaskManager singleton;
            return &singleton;
        }
};




#endif
