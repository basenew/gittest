#ifndef UP_CAN_PKG_H_H_H_
#define UP_CAN_PKG_H_H_H_


#include "can/can.h"

#include "can/can_pkg.h"

#include <boost/bind.hpp>
#include <boost/lockfree/queue.hpp>
#include <boost/thread.hpp>
#include <list>

class listQueue
{
    public:
        listQueue();
         ~listQueue();
        void insertQueue(CanPkg &package);
        bool getQueue(CanPkg *package,int timeOut);
        void setDebugInfoVal(bool val);

        bool resetQueue();
    
    private:
         std::list<CanPkg> canQueue;
         boost::mutex canMutex;
         boost::condition_variable_any condition;
         unsigned int list_size ;//only record top to 65535
         
         bool debug_can_pkg_;    
     
};


class upgradeCanPkg
{
    public:
        upgradeCanPkg(const char* dev_name);
        ~upgradeCanPkg();
        void init();
        bool packageToCanBus(CanPkg &package);
        bool getPackageFromQueue(CanPkg * package, int time_out);
        void packageFromCanBus(CanPkg &package);
        void loopReadPackage();

        void packgetGetPause();
        void packageGetRestart();
        
    private:
        boost::thread *canReadThread_;
        CanDevice cancom_;
        listQueue listQ;
        bool ispause_;

};








































#endif
