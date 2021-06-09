#ifndef CTimer_hpp__
#define CTimer_hpp__

#include <stdio.h>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <condition_variable>

class CTimer
{
public:
    CTimer(const std::string sTimerName = ""); // timer contructor with name
    ~CTimer();

    // start the timer
    // @ param msTime , delay run time (unit : ms)
    // @ param task , task function interface
    // @ param bLoop , whether or not the running is looped (default is once)
    // @ param async , whether or not it is async( default is set to async)
    // @ param true: whether or not it is ready to be executed, otherwise failed

    bool Start(unsigned int msTime, std::function<void()> task, bool bLoop = false, bool async = true);

    // cancel the timer
    // if the timer is synchronos (the cancel is failed)

    void Cancel();

    // synchronos executed once
    // @param msTime , delayed time (the unit is ms)
    // @param fun function interface or lambda function block
    // @param args
    // @return true: ready to be executed , otherwise failed

    template<typename callable, typename... arguments>
    bool SyncOnce(int msTime, callable&& fun, arguments&&... args) {
        std::function<typename std::result_of<callable(arguments...)>::type()> task(std::bind(std::forward<callable>(fun), std::forward<arguments>(args)...));
        return Start(msTime, task, false, false);
    }

    // async executed once
    // @param msTime , delayed to be executed time
    // @param fun , function and lambda interface
    // param args , input params
    // return true: ready to be executed , other wise failed

    template<typename callable, typename... arguments>
    bool AsyncOnce(int msTime, callable&& fun, arguments&&... args) {
        std::function<typename std::result_of<callable(arguments...)>::type()> task(std::bind(std::forward<callable>(fun), std::forward<arguments>(args)...));
        
        return Start(msTime, task, false);
    }

    // async executed once (executed after 1ms)
    // @param fun function interface and lambda function
    // @param args input arguments
    // return true: ready to be executed otherwise return false

    template<typename callable, typename... arguments>
    bool AsyncOnce(callable&& fun, arguments&&... args) {
        std::function<typename std::result_of<callable(arguments...)>::type()> task(std::bind(std::forward<callable>(fun), std::forward<arguments>(args)...));
        
        return Start(1, task, false);
    }

    // async executed loop
    // @param msTime delayed to be executed
    // @param fun function interface and lambda function block
    // @param args input arguments
    // return true : ready to be excuted, otherwise return false

    template<typename callable, typename... arguments>
    bool AsyncLoop(int msTime, callable&& fun, arguments&&... args) {
        std::function<typename std::result_of<callable(arguments...)>::type()> task(std::bind(std::forward<callable>(fun), std::forward<arguments>(args)...));
        
        return Start(msTime, task, true);
    }

    void DeleteThread(); // delete task thread
private:
	//void DeleteThread(); // delete task thread

public:
	int m_nCount = 0; // loop time

private:
	std::string m_sName; // timer name
	std::atomic_bool m_bExpired; // whether or not the task is expired
	std::atomic_bool m_bTryExpired; // whether or not trying to expire the loaded task
	std::atomic_bool m_bLoop; // whether or not is looped

	std::thread *m_Thread = nullptr;
	std::mutex m_ThreadLock;
	std::condition_variable_any m_ThreadCon;

};

#endif /* CTimer_hpp */
