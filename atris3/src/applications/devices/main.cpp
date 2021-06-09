#include <signal.h>
#include "utils/utils.h"
#include "flash_lamp.h"
#include "robot/robot.h"
#include "config/config.h"
#include <curl/curl.h>
#include "eeprom/eeprom.h"
#include "power_manager.h"
#include "chassis_manager.h"
#include "telecom/telecom.h"
#include "voip/voip.h"
#include "can_service/can_service.h"
#include "negotiates/negotiates.h"
#include "ultrasound/ultrasound.h"
#include "mobile_detector/mobile_detector.h"
#include "ptz/ptz_hk.h"
#include "router/router.h"
#include "imu/imu.h"
#include "tiny_ros/ros.h"
#include "ros/ros.h"
#include "log/log.h"
#include "peripheral_service/McuManager.h"
#include "dbcom/dbcom_signal.h"
#include "gps/gps.h"
#include "atris_msgs/PtzCtrlTest.h"
#include "atris_msgs/PtzControl.h"

// #define PTZCTRL_TEST 1

void signal_handler(int signum)
{
    log_warn("signal_handler signum=%d", signum);
    if(signum == SIGINT || signum == SIGABRT || signum == SIGSEGV){
        //TODO do something to save some information to disk.... 
        log_warn("exit signal received");
        exit(-1);
    }
}


#ifdef PTZCTRL_TEST
ros::ServiceClient g_ptz_control_srv_client;
void ptz_hk_testCallback(const atris_msgs::PtzCtrlTest::ConstPtr& msg)
{

    ROS_INFO("i heard ptzctrl_test msg");
    atris_msgs::PtzControl ptzctrl;
    ptzctrl.request.h_angle = msg->h_angle;
    ptzctrl.request.v_angle = msg->v_angle;
    ptzctrl.request.zoom = msg->zoom;

    if (msg->test_type == 1) //move_ptz
    {
        ptzctrl.request.cmd == atris_msgs::PtzControl::Request::PTZ_MOVE;
    }
    else if (msg->test_type == 2)
    {
        ptzctrl.request.cmd == atris_msgs::PtzControl::Request::PTZ_CAPTURE;
    }
    else if (msg->test_type == 3)
    {
        ptzctrl.request.cmd == atris_msgs::PtzControl::Request::PTZ_CAPTURE;
    }
    else
    {

    }

    g_ptz_control_srv_client.call(ptzctrl);
    // this_thread::sleep_for(milliseconds(1000));
    

    if (msg->test_type == 2)
    {
        if(!ptzctrl.response.visible_light_url.empty())
        {
            ROS_INFO("visible light url :%s", ptzctrl.response.visible_light_url.c_str());
            // _pinfo.visiblePicUrl = ptzctrl.response.visible_light_url;
        }
        else
        {
            ROS_INFO("visible light url is not gen");
	    }
        if(!ptzctrl.response.visible_light_url.empty())
        {
            ROS_INFO("visible light url :%s", ptzctrl.response.visible_light_url.c_str());
            // _pinfo.visiblePicUrl = ptzctrl.response.visible_light_url;
        }
        else
        {
             ROS_INFO("visible light url is not gen");
	    }
    }
    
}
#endif // PTZCTRL_TEST

int main(int argc, char *argv[]) {
    int ret = 0;
    ros::init(argc, argv, "devices");
    ros::NodeHandle n;
    tinyros::init("devices");

    signal(SIGPIPE, SIG_IGN);
    signal(SIGSEGV, signal_handler);
    signal(SIGINT, signal_handler);
    signal(SIGABRT, signal_handler);
    
    shm::iMemory_init();
    curl_global_init(CURL_GLOBAL_ALL);

    Utils * utils = Utils::get_instance();


    //ret = Eeprom::get_instance()->init();
    //if(ret < 0){
    //    log_error("Eeprom: init eeprom fail.");
    //}

    Robot::get_instance()->init();
    Propaganda::get_instance();
    Disperse::get_instance();

    TaskManager *taskmanager = TaskManager::get_instance();
    taskmanager->init();

    ChassisManager::get_instance()->init();

   // CanService *can = CanService::get_instance();
   // Ultrasound::get_instance()->init(can);
    //FlashLamp::get_instance()->init(can);

    //MobileDetect::get_instance()->init();
    Negotiates::get_instance();
    Ptz_hk::get_instance()->init();
    FlashLamp::get_instance()->init();
    Imu::get_instance()->init();
    PowerManager *pm = PowerManager::get_instance();
    ret = pm->init();
    if(ret < 0){
        log_error("init powermanager fail.");
    }

    McuManager *mcu_manager = McuManager::get_instance();
    ret = mcu_manager->init();
    if(ret < 0)
    {
        printf("mcu manager init\r\n");
    }

    DbComManager::get_instance();

    Gps::get_instance();

    ret = utils->init();
    if(ret < 0){
        log_error("Utils init fail.");
    }

   // Router::get_instance();
   // Telecom::get_instance();
   // Voip::get_instance();

#ifdef PTZCTRL_TEST
    g_ptz_control_srv_client = n.serviceClient<atris_msgs::PtzControl>(SRV_PTZ_CONTROL);
    ros::Subscriber sub = n.subscribe("/topic/ptz/hk/test", 1000, ptz_hk_testCallback);
#endif

    ros::MultiThreadedSpinner s(10);
    ros::spin(s);

    curl_global_cleanup();

    return 0;
}
