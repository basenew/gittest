#include "CTimer.hpp"
#include <future>
//#include "CPrintfLog.h"

CTimer::CTimer(const std::string sTimerName):m_bExpired(true), m_bTryExpired(false), m_bLoop(false)
{
    m_sName = sTimerName;
}

CTimer::~CTimer()
{
    m_bTryExpired = true;   // try to expired the task
    DeleteThread();
}

bool CTimer::Start(unsigned int msTime, std::function<void()> task, bool bLoop, bool async)
{
    if (!m_bExpired || m_bTryExpired) return false;  // task is not expired , we return false since the task may be in executed state
    m_bExpired = false;
    m_bLoop = bLoop;
    m_nCount = 0;

    if (async) 
    {
    	// async executed the task
        DeleteThread();
        m_Thread = new std::thread([this, msTime, task]() {
            if (!m_sName.empty()) {

            }
            
            while (!m_bTryExpired) {
                m_ThreadCon.wait_for(m_ThreadLock, std::chrono::milliseconds(msTime));  //休眠
                if (!m_bTryExpired) {
                    task();     //execute the task

                    m_nCount ++;
                    if (!m_bLoop) {
                        break;
                    }
                }
            }
            
            m_bExpired = true;      //task executed ok
            m_bTryExpired = false;  //in order to load the task ok next time
        });
    }
    else
    {
    	// execute the task synchronous
        std::this_thread::sleep_for(std::chrono::milliseconds(msTime));
        if (!m_bTryExpired) {
            task();
        }
        
        m_bExpired = true;
        m_bTryExpired = false;
    }
    
    return true;
}

void CTimer::Cancel()
{
    if (m_bExpired || m_bTryExpired || !m_Thread) {
        return;
    }
    
    m_bTryExpired = true;
}

void CTimer::DeleteThread()
{
    if (m_Thread) {
        m_ThreadCon.notify_all();   
        m_Thread->join();           
        delete m_Thread;
        m_Thread = nullptr;
    }
}