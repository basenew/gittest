#include <boost/thread/thread.hpp>
#include <sys/time.h>
#include <pthread.h>
#include "task_manager.h"
#include "log/log.h"

TaskManager::TaskManager()
{
    pthread_mutex_init(&mutex, NULL);
    pthread_condattr_setclock(&cattr, CLOCK_MONOTONIC);
    pthread_cond_init(&new_task_cond, NULL);
    pthread_cond_init(&delay_cond, &cattr);
    task_id = 0;
    new boost::thread(boost::bind(&TaskManager::task_exec, this));
}

int TaskManager::init()
{
    return 0;
}

/**
 * @brief post 添加立即执行的任务
 *
 * @param task
 *
 * @return 返回任务ID
 */
int TaskManager::post(Task &task)
{
    pthread_mutex_lock(&mutex);

    task_id ++;
    task.id = task_id;
    task.realtime = 0;
    task.type = TT_ONCE;

    if(task_list.size() == 0){
        //log_info("add new task:%d to empty list", task.id);
        task_list.push_back(task);
        ::pthread_cond_signal(&new_task_cond);
    }
    else{
        //log_info("add new task to list");
        task_list.push_back(task);
        task_list.sort();
        //show_task();
    }

    pthread_mutex_unlock(&mutex);

    return task_id;
}

/**
 * @brief post_delay 提交延时执行任务
 *
 * @param task 被执行的任务
 * @param delay_ms 延时时间，单位为毫秒
 *
 * @return 返回任务ID
 */
int TaskManager::post_delay(Task &task, int delay_ms)
{
    struct timespec now = {0, 0};

    pthread_mutex_lock(&mutex);

    task_id ++;
    task.id = task_id;
    task.type = TT_ONCE;
    task.cycle_time = 0;

    clock_gettime(CLOCK_MONOTONIC, &now);
    task.realtime = (now.tv_sec * 1000000LL + now.tv_nsec/1000) + delay_ms * 1000LL;
    //int64_t n = now.tv_sec * 1000000LL + now.tv_nsec/1000;
    //log_warn("add task(%d) realtime:%lld now:%lld delay:%d", task.id, task.realtime, n, delay_ms);

    if(task_list.size() == 0)
    {
        //log_info("add new task to empty list, delay:%d", delay_ms);
        task_list.push_back(task);
        ::pthread_cond_signal(&new_task_cond);
    }
    else
    {
        //log_info("add new task to list, delay:%d", delay_ms);
        task_list.push_back(task);
        task_list.sort();
        //show_task();
    }

    pthread_mutex_unlock(&mutex);

    return task_id;
}


/**
 * @brief post_cycle 添加周期性执行任务
 *
 * @param task 待执行的任务
 * @param cycle_time 执行任务的周期,单位为毫秒
 *
 * @return
 */
int TaskManager::post_cycle(Task &task, unsigned int cycle_time)
{
    struct timespec now;

    pthread_mutex_lock(&mutex);

    task_id ++;
    task.id = task_id;
    task.type = TT_REPEAT;
    task.cycle_time = cycle_time * 1000LL;

    clock_gettime(CLOCK_MONOTONIC, &now);
    task.realtime = (now.tv_sec * 1000000LL + now.tv_nsec/1000) + cycle_time;
    //log_warn("add task(%d) realtime:%lld delay:%d", task.id, task.realtime, cycle_time);

    if(task_list.size() == 0)
    {
        //log_info("add new task to empty list, delay:%d", delay_ms);
        task_list.push_back(task);
        ::pthread_cond_signal(&new_task_cond);
    }
    else
    {
        //log_info("add new task to list, delay:%d", delay_ms);
        task_list.push_back(task);
        task_list.sort();
        //show_task();
    }

    pthread_mutex_unlock(&mutex);

    return task_id;
}

/**
 * @brief del delete task
 *
 * @param task_id
 *
 * @return return 0 when delete task success, else return -1
 */
int TaskManager::del(int task_id)
{
    if(task_id < 0)
        return -1;

    pthread_mutex_lock(&mutex);

    for(std::list<Task>::iterator it = task_list.begin(); it != task_list.end(); it ++){
        if((*it).id == task_id){
            task_list.erase(it);
            pthread_mutex_unlock(&mutex);
            return 0;
        }
    }

    pthread_mutex_unlock(&mutex);

    return -1;
}

void TaskManager::task_exec()
{
    Task task;
    std::list<Task>::iterator task_it;
    struct timespec delay;
    struct timespec now;

    log_info("TaskManager task exec");
    while(true){
        pthread_mutex_lock(&mutex);
        while(task_list.size() == 0){
            pthread_cond_wait(&new_task_cond, &mutex);
        }

        for(;;){
            if(task_list.size() == 0)
                break;

            task_it = task_list.begin();

            clock_gettime(CLOCK_MONOTONIC, &now);
            int64_t nowtime = now.tv_sec * 1000000LL + now.tv_nsec/1000LL;
            int64_t delay_us = task_it->realtime - nowtime;
            if(delay_us <= 0){
                break;
            }

            int64_t sec = (now.tv_nsec/1000 + delay_us)/1000000LL;
            int64_t nsec = (now.tv_nsec/1000 + delay_us - sec*1000000LL) * 1000;
            delay.tv_sec = now.tv_sec + sec;
            delay.tv_nsec = nsec;

            int err = pthread_cond_timedwait(&delay_cond, &mutex, &delay);
            if(err == ETIMEDOUT)
            {
                clock_gettime(CLOCK_MONOTONIC, &now);
                break;
            }
        }

        if (task_list.size() > 0 && task_it != task_list.end()) {
            task = *task_it;
            if(task_it->type == TT_ONCE)
                task_list.erase(task_it);
            else if(task_it->type == TT_REPEAT){
                clock_gettime(CLOCK_MONOTONIC, &now);
                task_it->realtime = (now.tv_sec * 1000000LL + now.tv_nsec/1000) + task_it->cycle_time;
                task_list.sort();
            }

            //show_task();
            pthread_mutex_unlock(&mutex);

            if(!task.cb.empty()){
                task.cb();
            }
        } else {
            pthread_mutex_unlock(&mutex);
        }
    }
}

void TaskManager::show_task()
{
    for(std::list<Task>::iterator it = task_list.begin(); it != task_list.end(); it ++){
        log_info("task id:%d time:%lld", it->id, it->realtime);
    }
}
