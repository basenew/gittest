/******************************************************************************

  Copyright (C), 2001-2011,UBT.CO.

 ******************************************************************************
  File Name     : upgradeCanPkg.cpp
  Version       : Initial Draft
  Author        : marty.gong@ubtrobot.com
  Created       : 2018/3/26
  Last Modified :
  Description   : deal with can package recv and send 

******************************************************************************/

/*----------------------------------------------*
 * include files                           *
 *----------------------------------------------*/
#include "include/upgradeCanPkg.h"

/*----------------------------------------------*
 * external variables                           *
 *----------------------------------------------*/

/*----------------------------------------------*
 * external routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * internal routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * project-wide global variables                *
 *----------------------------------------------*/

/*----------------------------------------------*
 * module-wide global variables                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * constants                                    *
 *----------------------------------------------*/

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/
#define MAX_LIST_QUE_SIZE  10000

upgradeCanPkg::upgradeCanPkg(const char* dev_name):cancom_(dev_name),ispause_(false)
{
    
}


upgradeCanPkg::~upgradeCanPkg()
{
    

}




void upgradeCanPkg::init()
{

    cancom_.can_open();

    canReadThread_ = new boost::thread(boost::bind(&upgradeCanPkg::loopReadPackage,this));
    
}

void upgradeCanPkg::loopReadPackage()
{
    CanPkg pkg;
    while(true)
    {
        if(true == ispause_)
        {
            usleep(100*1000);
            continue;
        }
        memset(&pkg,0,sizeof(CanPkg));
        if(cancom_.can_read(&pkg,1000) < 0)
        {
            log_warn("can read timeout.try reconnect device.");
            cancom_.can_close();
            cancom_.can_open();
            continue;
        }
        packageFromCanBus(pkg);
    }
}
bool upgradeCanPkg::packageToCanBus(CanPkg &package)
{
       listQ.resetQueue();
       cancom_.can_write(&package,package.head.id.size+1);
       return true;
}

bool  upgradeCanPkg::getPackageFromQueue(CanPkg * package, int time_out)
{
    return listQ.getQueue(package, time_out);
}


void upgradeCanPkg::packageFromCanBus(CanPkg & package)
{
    listQ.insertQueue(package);
}


void upgradeCanPkg::packgetGetPause()
{
     if(true == ispause_)return ;
     ispause_ = true;
     log_info("pause the package get from can bus");
}

void upgradeCanPkg::packageGetRestart()
{
  if(false == ispause_)return ;
  ispause_ = false;
  log_info("restart the package get from can bus");
  sleep(1);//wait for all package not losing.
}



//list functions   modified by marty.gong@ubtrobot.com:2018-3-26-18:2:32 



listQueue::listQueue():list_size(0)
{
    debug_can_pkg_ = false;
}


listQueue::~listQueue()
{

}
void  listQueue::setDebugInfoVal(bool val)
{
    debug_can_pkg_ = val;
}

void listQueue::insertQueue(CanPkg& package)
{

    if(package.data[0] < 0xba) return ;
    if(true == debug_can_pkg_)
    {
         if(package.data[0] >= 0xbb)
		 {
            printf("in size:%d mode:%d,channel:%.2x bus:%d==data =%.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x \n",package.head.id.size,package.head.id.mode,package.head.id.channel,package.head.id.can,
				   package.data[0],package.data[1],package.data[2],package.data[3],package.data[4],package.data[5],package.data[6],package.data[7]);
         }
    }
    if(list_size>MAX_LIST_QUE_SIZE)
    {
      //  printf("recevie exceed max size.");
        return ;
    }

    //如果 lock失败,可以等一段时间      保证尽可能的不丢包modified by marty.gong@ubtrobot.com:2017-11-25-11:17:0 
     clockid_t time_end;
    //2ms   modified by marty.gong@ubtrobot.com:2017-11-25-11:21:16 
    time_end = clock()+2000;
    while(true)
    {

        if(canMutex.try_lock())
        {
            canQueue.push_back(package);
            list_size++;
            canMutex.unlock();
            condition.notify_one();
            return ;
        }
        if(clock() > time_end)
        {
            return ;
        }
        
    }
}


bool listQueue::getQueue(CanPkg *package,int time_out)
{
     clockid_t time_end;

     if(NULL == package) return false;
    if((time_out < 0) && (canQueue.empty()))
    {
        condition.wait(canMutex);
    }else if(canQueue.empty())
   {

        boost::mutex::scoped_lock lock(canMutex);
        if (!condition.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(time_out)))
        {
            return false;
        }
    }
   if(canQueue.empty())
   {
        return false;
   }
    time_end = clock()+300;
    while(true)
    {
        if(canMutex.try_lock())
        {
            *package = canQueue.front();
             canQueue.erase(canQueue.begin());//*package);
             list_size--;
             canMutex.unlock();
             return true;

        }
        if(clock() > time_end) break;

    }
    return false;
}



bool listQueue::resetQueue()
{
    
    CanPkg package;
    while (getQueue(&package,0))
	{
		;
	}
    return true;
}






























































