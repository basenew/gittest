#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

#include "time.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include <json/json.h>
#include "ros/ros.h"
#include "platform/chassis/ChassisDef.h"
#include "power_manager/power_manager.h"
#include "atris_defines.h"
#include "atris_msgs/RobotInfo.h"
#include "log/log.h"
class ControlModule{

public:
    ControlModule() {
        diag_info_pub_ = nh_.advertise<atris_msgs::RobotInfo>(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100);
        control_thread_ = NULL;
        control_thread_exit_ = false;
        sys_tick = 0;
        control_module_init();
    }
    ~ControlModule(){
        control_thread_exit_ = true;

        if(control_thread_){
            control_thread_->interrupt();
            control_thread_->join();
            delete control_thread_;
        }
    }

    /*************************************************************
      Description:用于更改当前控制
      Input      :控制优先级
      return     :1-表示获取控制权成功，0-表示获取控制权失败
    *************************************************************/
    bool control_change(Control_Owner owner)
    {
        boost::lock_guard<boost::mutex> lock(control_lock);
        Control_Owner old_owner = get_current_owner();
        if(compare_current_owner(owner)){
            control_ticks[owner] = control_get_system_tick();
            set_current_owner(owner);
             save_priority_diag((Control_Owner)owner);
            if(old_owner != owner){

                switch(owner){
                    case REMOTE:
                        log_info("[control module]set remote control!");
                        break;
                    case WEB_CTRL:
                        log_info("[control module]set web control!");
                        break;                   
                    case SHTTPD:
                        log_info("[control module]set shttpd control!");
                        break;
                    case PC_CTRL:
                        log_info("[control module]set pc control!");
                        break;
                    case RECHARGE:
                        log_info("[control module]set recharge control!");
                        break;
                    case SELF_TEST:
                        log_info("[control module]set self test control!");
                        break;
                    case GS:
                        log_info("[control module]set gs control!");
                        break;
                    case TIME_OUT:
                        log_info("[control module]set time out control!");
                        break;
                }
            }
            return true;
        }
        return false;
    }

    void save_priority_diag(int priority)
    {
        Json::FastWriter fw;
        Json::Value root;
        atris_msgs::RobotInfo rbtInfo;
        std::string str;
        static int old = -10;
        if(old == priority)
            return;
        if(DF_CHASSIS_PRIORITY_TTS)
            switch(priority){
                case REMOTE:
                    PowerManager::get_instance()->sendTtstext(TTSStrings::TTS_KEY_REMOTE_CONTROL_CHASSIS);
                    break;
                case SHTTPD:
                    PowerManager::get_instance()->sendTtstext(TTSStrings::TTS_KEY_SHTTPD_CONTROL_CHASSIS);
                    break;
                case PC_CTRL:
                    PowerManager::get_instance()->sendTtstext(TTSStrings::TTS_KEY_PC_CONTROL_CHASSIS);
                    break;
                case RECHARGE:
                    PowerManager::get_instance()->sendTtstext(TTSStrings::TTS_KEY_RECHARGE_CONTROL_CHASSIS);
                    break;
                case SELF_TEST:
                    PowerManager::get_instance()->sendTtstext(TTSStrings::TTS_KEY_TEST_CONTROL_CHASSIS);
                    break;
                case GS:
                    PowerManager::get_instance()->sendTtstext(TTSStrings::TTS_KEY_NAVIGATION_CONTROL_CHASSIS);
                    break;
                case TIME_OUT:
                    PowerManager::get_instance()->sendTtstext(TTSStrings::TTS_KEY_TIMEOUT_CONTROL_CHASSIS);
                    break;

            }

        switch(priority){
            case REMOTE:
                str = "remote";
                break;
            case WEB_CTRL:
                str = "web";
                break;           
            case SHTTPD:
                str = "shttpd";
                break;
            case PC_CTRL:
                str = "pc";
                break;
            case RECHARGE:
                str = "recharge";
                break;
            case SELF_TEST:
               str = "test";
                break;
            case GS:
                str = "gs";
                break;
            case TIME_OUT:
               str = "time out";
                break;

        }
        
        old = priority;
        root["robot_info"]["chassis_driver"]["current_control_priority"] = str;
        rbtInfo.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo);
    }

    /*************************************************************
      Description:用于初始化控制模块初始化
      Input      :
      return     :
    *************************************************************/
    void control_module_init(void)
    {
        memset(control_status, 0, sizeof(control_status));
        memset(control_ticks, 0, sizeof(control_ticks));
        control_status[0] = 1;
        for(int i=0; i<NUMBER_OF_PRIORITY; i++){
            control_ticks_out[i] = 2000;
        }
        control_thread_ = new boost::thread(boost::bind(&ControlModule::control_module_thread, this));
    }
    /*************************************************************
      Description:获取当前控制状态
      Input      :none
      return     :Control_Owner
    *************************************************************/
    Control_Owner get_current_owner(void)
    {
        Control_Owner current_o = REMOTE;
        for(uint8_t i =0; i<NUMBER_OF_PRIORITY; i++){
            if(control_status[i] == 1){
                current_o = (Control_Owner)i;
                break;
            }
        }
        return current_o;
    }
private:
    unsigned long control_get_system_tick(void)
    {
//        struct timespec ts;
//        clock_gettime(CLOCK_MONOTONIC, &ts);
//        return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
        return sys_tick;
    }

    void set_current_owner(Control_Owner owner)
    {
        memset(control_status, 0, NUMBER_OF_PRIORITY);
        control_status[owner] = 1;
    }

    uint8_t control_is_timeout(Control_Owner owner)
    {
        if(control_get_system_tick() - control_ticks[owner] >= control_ticks_out[owner]){
            return 1;
        }
        return 0;
    }

    uint8_t compare_current_owner(Control_Owner owner)
    {
        uint8_t i;
        for(i =0; i< owner; i++){
            if(control_status[i] == 1){
                return 0;
            }
        }
        return 1;
    }
    /*************************************************************
      Description:用于while（1）中持续执行
      Input      :
      return     :
    *************************************************************/
    void control_process(void)
    {
        boost::lock_guard<boost::mutex> lock(control_lock);
        uint8_t i;
        if(control_status[NUMBER_OF_PRIORITY-1] == 1){
//            no control
        }
        for(i =0; i< NUMBER_OF_PRIORITY-1; i++){
            if(control_is_timeout((Control_Owner)i)){
                if(control_status[i] ==1){
                    set_current_owner((Control_Owner)(i+1));

                    switch((Control_Owner)(i+1)){
                        case REMOTE:
                            log_info("[control module]time out set remote control!");
                            break;
                        case WEB_CTRL:
                            log_info("[control module]time out set remote control!");
                            break;
                        case SHTTPD:
                            log_info("[control module]time out set shttpd control!");
                            break;
                        case PC_CTRL:
                            log_info("[control module]time out set pc control!");
                            break;
                        case RECHARGE:
                            log_info("[control module]time out set recharge control!");
                            break;
                        case SELF_TEST:
                            log_info("[control module]time out set self test control!");
                            break;
                        case GS:
                            log_info("[control module]time out set gs control!");
                            break;
                        case TIME_OUT:
                            log_info("[control module]time out set time out control!");
                            break;
                    }
                    return;
                }
            }
        }
    }

    void control_module_thread(void)
    {
        while(!control_thread_exit_)
        {
            sys_tick += 100;
            control_process();
            usleep(1000 *100);
        }
    }

    uint8_t         control_status[NUMBER_OF_PRIORITY];
    unsigned long   control_ticks[NUMBER_OF_PRIORITY];
    unsigned long   control_ticks_out[NUMBER_OF_PRIORITY];
    boost::mutex    control_lock;
    bool control_thread_exit_;
    boost::thread *control_thread_;
    unsigned long  sys_tick;
    ros::NodeHandle nh_;
    ros::Publisher diag_info_pub_;
};

#endif
