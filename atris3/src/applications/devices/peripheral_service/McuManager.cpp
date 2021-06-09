
/*
 * McuManager.cpp
 *
 *  Created on: 2020-10-14
 *      Author: jinzhongxi
 */
//
#include "McuManager.h"
#include "tiny_ros/atris_msgs/CanPkg.h"
#include "atris_msgs/SignalMessage.h"
#include "utils/utils.h"
#include "task_manager/task_manager.h"
#include <thread>
#include <time.h>

#define SWUPGRADE_ICP_SUCCESS "success"
#define SWUPGRADE_ICP_FAILED "failed"
#define SWUPGRADE_ICP_STANDBY "standby"
#define SWUPGRADE_ICP_PROGRESS "progress"

#define SWUPGRADE_VERSION_FILE "/home/atris/atris_app/config/versionInfo.txt"

#define SYS_MONITOR_CMD_SEND(_duration_) \
{\
    std::unique_lock<std::mutex> map_lock(sys_monitor_map_mutex_); \
    sys_monitor_resp_map_[request.cmd + 1] = SysMonRespPtr(new SysMonResp());\
    sys_monitor_resp_map_[request.cmd + 1]->cond = SysMonRespCondPtr(new std::condition_variable()); \
    map_lock.unlock(); \
    std::unique_lock<std::mutex> resp_lock(sys_monitor_resp_map_[request.cmd + 1]->mutex); \
    sys_monitor_service_pub_.publish(&request); \
    if (_duration_ > 0) { \
        if (sys_monitor_resp_map_[request.cmd + 1]->cond->wait_until(resp_lock, std::chrono::system_clock::now() + \
            std::chrono::milliseconds(_duration_ * 1000)) == std::cv_status::timeout) { \
            log_warn("system monitor cmd request id : [%d] timeout!!!", request.cmd); \
            resp_lock.unlock(); \
            map_lock.lock(); \
            sys_monitor_resp_map_.erase(request.cmd + 1); \
            map_lock.unlock(); \
            return CTRL_REQ_TIMEOUT; \
        } \
    } else { \
        sys_monitor_resp_map_[request.cmd + 1]->cond->wait(resp_lock); \
    } \
    if (sys_monitor_resp_map_[request.cmd + 1]->resp.cmd == (request.cmd + 1)) { \
        response = sys_monitor_resp_map_[request.cmd + 1]->resp; \
    } \
    resp_lock.unlock(); \
    map_lock.lock(); \
    sys_monitor_resp_map_.erase(request.cmd + 1); \
    map_lock.unlock(); \
}

#define CHASSIS_CONTROLLER_CMD_SEND(_duration_) \
{\
    std::unique_lock<std::mutex> map_lock(chassis_controller_map_mutex_); \
    chassis_controller_resp_map_[request.cmd + 1] = ChassisCtrlRespPtr(new ChassisCtrlResp());\
    chassis_controller_resp_map_[request.cmd + 1]->cond = ChassisCtrlRespCondPtr(new std::condition_variable()); \
    map_lock.unlock(); \
    std::unique_lock<std::mutex> resp_lock(chassis_controller_resp_map_[request.cmd + 1]->mutex); \
    chassis_controller_service_pub_.publish(&request); \
    if (_duration_ > 0) { \
        if (chassis_controller_resp_map_[request.cmd + 1]->cond->wait_until(resp_lock, std::chrono::system_clock::now() + \
            std::chrono::milliseconds(_duration_ * 1000)) == std::cv_status::timeout) { \
            log_warn("chassis controller cmd request id : [%d] timeout!!!", request.cmd); \
            resp_lock.unlock(); \
            map_lock.lock(); \
            chassis_controller_resp_map_.erase(request.cmd + 1); \
            map_lock.unlock(); \
            return CTRL_REQ_TIMEOUT; \
        } \
    } else { \
        chassis_controller_resp_map_[request.cmd + 1]->cond->wait(resp_lock); \
    } \
    if (chassis_controller_resp_map_[request.cmd + 1]->resp.cmd == (request.cmd + 1)) { \
        response = chassis_controller_resp_map_[request.cmd + 1]->resp; \
    } \
    resp_lock.unlock(); \
    map_lock.lock(); \
    chassis_controller_resp_map_.erase(request.cmd + 1); \
    map_lock.unlock(); \
}

McuManager::McuManager()
  : sys_monitor_resp_sub_(TOPIC_MONITOR_UPGRADE_RESP, &McuManager::on_recv_sys_monitor_resp, this)
  , sys_monitor_service_pub_(TOPIC_MONITOR_UPGRADE_REQ, new tinyros::atris_msgs::CanPkg())
  , chassis_controller_report_sub_(TOPIC_CHASSIS_UPGRADE_RESP, &McuManager::on_recv_chassis_controller_message, this)
  , chassis_controller_service_pub_(TOPIC_CHASSIS_UPGRADE_REQ, new tinyros::atris_msgs::CanPkg())
  , emergency_button_status_(-1)
  , collision_bumper_status_(-1)
  , brake_enable_status_(-1)
  , remote_status_(DF_REMOTE_OFF_LINE)
  , ptz_up_down_state_(PTZ_VERTICAL_MOVE_IDLE)
  , ptz_left_right_state_(PTZ_HORIZONTAL_MOVE_IDLE)
  , brake_state_(REMOTE_ENABLE_BRAKE_NULL)
  , ptz_brush_state_(PTZ_BRUSH_STATE_OFF)
  , supplement_light_state_(PTZ_SUPPLEMENT_LIGHT_OFF)
  , is_power_off_(false)
  , is_charger_standby_(false)
  , is_recharge_standby_(false)
  , vra_value_last_(-1)
  , vrb_value_last_(-1)
  , vra_stable_count_(0)
  , vrb_stable_count_(0)
  , vra_stable_send_once_(0)
  , vrb_stable_send_once_(0)
  , swh_stable_send_once_(0)
  , swh_start_to_count_(false)
  , vra_stable_state_(true)
  , vrb_stable_state_(true)
  , vra_filter_out_val_(0)
  , vrb_filter_out_val_(0)
  , chassis_sw_version_("")
  , chassis_base_hw_version_(0)
  , chassis_core_hw_version_(0)
  , monitor_sw_version_("")
  , monitor_base_hw_version_(0)
  , monitor_core_hw_version_(0)
  , bms_hw_version_("")
  , bms_sw_version_("")
  , imx_version_("")
  , isMonitorMcuVerReceived(false)
  , isChassisMcuVerReceived(false)
  , isBMSSwVerReceived(false)
  , isBMSHwVerReceived(false)
  , robot_mode_(0)
  , anti_drop_status_(0)
  , ultra_info_msg_recv_last_(0)
  , recv_anti_drop_last_(0)
  , recv_ads_last_(0)
  , diag_status(0)
{
    // tinyros used to communicate with mcu
    tinyros::nh()->subscribe(sys_monitor_resp_sub_);
    tinyros::nh()->advertise(sys_monitor_service_pub_);
    tinyros::nh()->subscribe(chassis_controller_report_sub_);
    tinyros::nh()->advertise(chassis_controller_service_pub_);

    // ros standard
    anti_drop_pub_ = nh_.advertise<atris_msgs::AntiDrop>(TOPIC_ANTI_DROP_STATE, 100);
    diag_info_pub_ = nh_.advertise<atris_msgs::RobotInfo>(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100);
    ptz_ctrl_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_REQUEST_MESSAGE, 100);
    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &McuManager::on_recv_peripheral_ctrl, this);
    charge_response_pub_ = nh_.advertise<atris_msgs::PowerChargeCmd>(TOPIC_POWER_CHARGE_CMD_RESP_MESSAGE, 100);
    charge_request_sub_ = nh_.subscribe(TOPIC_POWER_CHARGE_CMD_REQ_MESSAGE, 100, &McuManager::on_recv_navigation_charge_request, this);
    get_sw_version_srv_ = nh_.advertiseService(SRV_GET_SW_VERSION, &McuManager::doGetSwVersion, this);
    get_robot_standby_mode_srv = nh_.advertiseService(SRC_GET_STANDBY_MODE, &McuManager::doGetRobotStandbyStatus, this);
    mcu_upload_response_pub_ = nh_.advertise<atris_msgs::McuLogCtrl>(TOPIC_DIAG_MCU_LOG_MESSAGE_RESP,100);
    mcu_upload_request_sub_ = nh_.subscribe(TOPIC_DIAG_MCU_LOG_MESSAGE_REQ, 100 , &McuManager::on_recv_mcu_upload_log_request, this);
    robot_run_mode_pub_ =  nh_.advertise<atris_msgs::RobotRunMode>(TOPIC_ROBOT_RUN_MODE, 100);
    peripheral_control_srv_ = nh_.advertiseService(SRV_PERIPHERAL_CONTROL, &McuManager::on_recv_peripheral_device_ctrl, this);
    aisound_tts_pub_ = nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    rbt_sn_ = shmrbt.robot.sn;
    log_info("%s mcu manager get robot sn: %s", __FUNCTION__, rbt_sn_.c_str());
    imx_version_ = getImxVersion();
    #if 1
    std::thread thd(&McuManager::do_get_software_version_task, this);
    thd.detach();
    #endif

    std::thread device_check_thd(&McuManager::checkDeviceError, this);
    device_check_thd.detach();
}

int McuManager::init()
{
    return 0;
}

// do query firmware info in cycle task
int McuManager::do_get_software_version_task(void)
{
    int ultra_ver;
    int fan_speed;
    int fan1_error;
    int fan2_error;
    int iRet;
    bool isRun = true;
    bool getAllSwVer = false;
    std::string upgrad_status;

    log_info("%s query software version thread start",__FUNCTION__);

    while(isRun && !getAllSwVer)
    {

        upgrad_status = getICPUpgradeStatus();
        if(upgrad_status != SWUPGRADE_ICP_STANDBY)
        {
            log_warn("%s software upgrading , not ok to query software version from mcu , upgrad_status = %s",__FUNCTION__, upgrad_status.c_str());
            log_warn("%s query software version later in 5 secs", __FUNCTION__);
            sleep(5);
            continue;
        }
        else
        {
            log_info("%s ok to query software version , upgrad_status = %s",__FUNCTION__, upgrad_status.c_str());
        }

        usleep(1000 * 1000);

        if(!isMonitorMcuVerReceived)
        {
            log_info("monitor version not received, query monitor version from mcu\r\n");
            queryMonitorMcuVersionInfo();
        }

        usleep(1000 * 1000);

        if(!isChassisMcuVerReceived)
        {
            log_info("chassis version not received, query chassis version from mcu\r\n");
            queryChassisMcuVersionInfo();
        }

        usleep(1000 * 1000);

        if(!isBMSHwVerReceived)
        {
            log_info("%s query bms hw version",__FUNCTION__);
            queryBMSHardWareVersionInfo();
        }

        usleep(1000 * 1000);
        
        if(!isBMSSwVerReceived)
        {
            log_info("%s query bms sw version",__FUNCTION__);
            queryBMSSoftWareVersionInfo();
        }

        usleep(1000 * 1000);

        if(isMonitorMcuVerReceived && isChassisMcuVerReceived && isBMSSwVerReceived && isBMSHwVerReceived)
        {
            getAllSwVer = true;
        }
        else
        {
            log_info("not getting all product info , chassis mcu version info get = %d , bms sw version info get = %d , bms hw version info get = %d", isChassisMcuVerReceived?1:0, isBMSSwVerReceived?1:0, isBMSHwVerReceived?1:0);
        }
    }

    log_info("%s query software version thread finish",__FUNCTION__);

    return 0;
}

// request getting monitor mcu version info
// cmd id = 291
void McuManager::queryMonitorMcuVersionInfo(void)
{
    log_info("%s ************* query monitor version from mcu **********\r\n",__FUNCTION__);
    tinyros::atris_msgs::CanPkg request;
    request.cmd = QUERY_MONITOR_VER_REQ;
    sys_monitor_service_pub_.publish(&request);
}

// request getting chassis mcu version info
// cmd id = 191
void McuManager::queryChassisMcuVersionInfo(void)
{
    log_info("%s ************* query chassis version from mcu **********\r\n",__FUNCTION__);
    tinyros::atris_msgs::CanPkg request;
    request.cmd = QUERY_CHASSIS_VER_REQ;
    chassis_controller_service_pub_.publish(&request);
}

// request getting bms hard version info
// cmd id = 193
void McuManager::queryBMSHardWareVersionInfo(void)
{
    log_info("%s",__FUNCTION__);
    tinyros::atris_msgs::CanPkg request;
    request.cmd = QUERY_BMS_HW_VER_REQ;
    chassis_controller_service_pub_.publish(&request);
}

// request getting bms soft version info
// cmd id = 195
void McuManager::queryBMSSoftWareVersionInfo(void)
{
    log_info("%s",__FUNCTION__);
    tinyros::atris_msgs::CanPkg request;
    request.cmd = QUERY_BMS_SW_VER_REQ;
    chassis_controller_service_pub_.publish(&request);
}

// recv system monitor message(active report) 
// if the message is not active report , it is synchronous response message
void McuManager::on_recv_sys_monitor_resp(const tinyros::atris_msgs::CanPkg &msg)
{
    //log_info("recv sys monitor report msg!!! cmd id = %d", msg.cmd);

    // for debugging purpose
    // just simply print the received message
    if(MCU_REPORT_MSG_DBG)
    {
        //printCanPkgMsg(msg);
    }

    if(0x02 == msg.cmd)
    {
        // do not deal with upgrade message
        return;
    }

    // parse command information
    if(VOLTAGE_MONITOR_INFO == msg.cmd)
    {
        processVoltageInfoMsg(msg);
    }
    else if(CURRENT_MONITOR_INFO == msg.cmd)
    {
        processCurrentInfoMsg(msg);
    }
    else if(ULTRA_SOUND_MONITOR_INFO == msg.cmd)
    {
        processUltraMsg(msg);
    }
    else if(TEMP_HUMI_MONITOR_INFO == msg.cmd)
    {
        processTempHumiMsg(msg);
    }
    else if(ANTI_DROP_MONITOR_INFO == msg.cmd)
    {
        //log_info("recv anti drop message");
        processAntiDropMsg(msg);
    }
    else if(QUERY_MONITOR_VER_RESP == msg.cmd)
    {
        processMonitorMcuVersion(msg);
    }
    else if(MONITOR_LOG_FILE_GET_RESP == msg.cmd)
    {
        processMonitorUploadLogResp(msg);
    }
    else
    {
        doProcSysMonitorCmdResp(msg);
    }
}

// check if the request command id is already in the map
bool McuManager::removeSysMonDupRequest(int msg_id)
{
    std::unique_lock<std::mutex> lock(sys_monitor_map_mutex_);
    if (sys_monitor_resp_map_.count(msg_id + 1) > 0) 
    {
        log_info("%s found corresponding request duplicate message waited in map , request cmd id : %d\r\n", __FUNCTION__, msg_id);
        sys_monitor_resp_map_.erase(msg_id + 1);
        return true;
    }

    log_info("%s ok to wait message request in response map", __FUNCTION__);
    return false;
}

// get response from system monitor , just signal the corresponding condition variable
void McuManager::doProcSysMonitorCmdResp(const tinyros::atris_msgs::CanPkg & msg)
{
    std::unique_lock<std::mutex> lock(sys_monitor_map_mutex_);
    //sys_monitor_resp_ = msg;
    if (sys_monitor_resp_map_.count(msg.cmd) > 0) 
    {
        log_info("found corresponding request waited command , resp cmd id : %d\r\n", msg.cmd);
        std::unique_lock<std::mutex> resp_lock(sys_monitor_resp_map_[msg.cmd]->mutex);
        sys_monitor_resp_map_[msg.cmd]->resp = msg;
        sys_monitor_resp_map_[msg.cmd]->cond->notify_all();
    }
    else
    {
        log_error("not found corresponding request waited in map , id : %d", msg.cmd);
    }
}

// query indicator light device status
// for internal use of the class to query the state of the indicator light
int McuManager::queryIndicatorLight(int & style, int & color, int & brightness)
{
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = INDICATOR_LIGHT_REQ;
    request.data_i[0] = 0; // query command

    SYS_MONITOR_CMD_SEND(5);

    log_info("%s query inicator light info success... style : %d , color : %d , brightness %d", __FUNCTION__, response.data_i[0], response.data_i[1], response.data_i[2]);

    style = response.data_i[0];
    color = response.data_i[1];
    brightness = response.data_i[2];

    return CTRL_REQ_SUCCESS;
}


// for external use to query the indicator light state
int McuManager::queryIndicatorLightState(int & style, int & color, int & brightness)
{
    std::unique_lock<std::mutex> lock(indicator_light_mutex_);

    bool bRet;

    bRet = removeSysMonDupRequest(INDICATOR_LIGHT_REQ);
    if(bRet)
    {
        log_warn("%s indicator light request found and remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s indicator light request not in map",__FUNCTION__);
    }

    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;


    request.cmd = INDICATOR_LIGHT_REQ;
    request.data_i[0] = 0; // query command

    SYS_MONITOR_CMD_SEND(5);

    log_info("%s query inicator light info success... style : %d , color : %d , brightness %d", __FUNCTION__, response.data_i[0], response.data_i[1], response.data_i[2]);

    style = response.data_i[0];
    color = response.data_i[1];
    brightness = response.data_i[2];

    return CTRL_REQ_SUCCESS;
}

// set indicator light style
// query the indicator light first , and don't change other property of the light style
int McuManager::setIndicatorLightStyle(int light_style)
{
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);
    // multithread protect lock
    std::unique_lock<std::mutex> lock(indicator_light_mutex_);

    // first judge if the light style command is in range 
    if(light_style < 0 || light_style > 2)
    {
        log_error("indicator light style input parameter invalid!!!");
        return CTRL_REQ_PARAM_INVALID;
    }
    else
    {
        log_info("%s set indicator light style to %s",__FUNCTION__, get_light_style_str(light_style).c_str());
    }

    int style, color, brightness;
    int control;
    int iRet;
    bool bRet;

    bRet = removeSysMonDupRequest(INDICATOR_LIGHT_REQ);
    if(bRet)
    {
        log_warn("%s indicator light request found and remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s indicator light request not in map",__FUNCTION__);
    }

    // query the indicator light style first
    iRet = queryIndicatorLight(style, color, brightness);
    if(0 > iRet)
    {
        log_error("%s query indicator light style failed!!! ret : %d",__FUNCTION__, iRet);
        return iRet;
    }
    else
    {
        log_info("%s light style = %d, light color = %d , light brightness = %d",__FUNCTION__, style, color, brightness);
    }

    if(style == light_style)
    {
    	log_warn("%s indicator light style already at : %d\r\n",__FUNCTION__, style);
    	return CTRL_REQ_SUCCESS;
    }

    // set the indicator light style according to input param
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = INDICATOR_LIGHT_REQ;
    request.data_i[0] = 1; // set command
    request.data_i[1] = light_style;
    request.data_i[2] = color;
    request.data_i[3] = brightness;
    request.data_i[4] = 1; // get control from mcu

    SYS_MONITOR_CMD_SEND(5);

    indicator_light_style_ = response.data_i[0];
    indicator_light_color_ = response.data_i[1];
    indicator_light_brightness_ = response.data_i[2];

    if(light_style != response.data_i[0])
    {
        log_error("%s set indicator light style to %d failed, response light style : %d!!!",__FUNCTION__, light_style, response.data_i[0]);
        return CTRL_REQ_INNER_ERROR;
    }

    log_info("%s set indicator light style to %d success, response light style : %d ...",__FUNCTION__, light_style, response.data_i[0]);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

    return CTRL_REQ_SUCCESS;
}

// set indicator light style
// query the indicator light first , and don't change other property of the light style
int McuManager::setIndicatorLightColor(int light_color)
{
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);
    // multithread protect lock

    std::unique_lock<std::mutex> lock(indicator_light_mutex_);

    // first judge if the light style command is in range 
    if(light_color < 1 || light_color > 7)
    {
        log_error("indicator light color input parameter invalid!!!");
        return CTRL_REQ_PARAM_INVALID;
    }
    else
    {
        log_info("%s set indicator light color to %s",__FUNCTION__, get_light_color_str(light_color).c_str());
    }

    int style, color, brightness;
    int iRet;
    bool bRet;

    bRet = removeSysMonDupRequest(INDICATOR_LIGHT_REQ);
    if(bRet)
    {
        log_warn("%s indicator light request found and remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s indicator light request not in map",__FUNCTION__);
    }

    // query the indicator light style first
    iRet = queryIndicatorLight(style, color, brightness);
    if(0 > iRet)
    {
        log_error("%s query indicator light style failed!!!",__FUNCTION__);
        return iRet;
    }
    else
    {
        log_info("%s light style = %d, light color = %d , light brightness = %d",__FUNCTION__, style, color, brightness);
    }

    if(color == light_color)
    {
    	log_warn("%s indicator light color already at : %d\r\n",__FUNCTION__, color);
    	return CTRL_REQ_SUCCESS;
    }

    // set the indicator light color according to input param
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = INDICATOR_LIGHT_REQ;
    request.data_i[0] = 1; // set command
    request.data_i[1] = style;
    request.data_i[2] = light_color;
    request.data_i[3] = brightness;
    request.data_i[4] = 1; // get control from mcu

    SYS_MONITOR_CMD_SEND(5);

    indicator_light_style_ = response.data_i[0];
    indicator_light_color_ = response.data_i[1];
    indicator_light_brightness_ = response.data_i[2];

    if(light_color != response.data_i[1])
    {
        log_error("%s set indicator light color to %d failed , response light color = %d!!!",__FUNCTION__, light_color, response.data_i[1]);
        return CTRL_REQ_INNER_ERROR;
    }

    log_info("%s set indicator light color to %d success... , response light color = %d",__FUNCTION__, light_color , response.data_i[1]);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

    return CTRL_REQ_SUCCESS;
}

// set indicator light style
// query the indicator light first , and don't change other property of the light style
int McuManager::setIndicatorLightBrightness(int light_brightness)
{
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);
    // multithread protect lock

    std::unique_lock<std::mutex> lock(indicator_light_mutex_);
    // first judge if the light brightness command is in range 
    if(light_brightness < 0 || light_brightness > 100)
    {
        log_error("indicator light brightness input parameter invalid!!!");
        return CTRL_REQ_PARAM_INVALID;
    }
    else
    {
        log_info("%s set indicator light brightness to %d percent",__FUNCTION__, light_brightness);
    }

    int style, color, brightness;
    int iRet;
    bool bRet;

    bRet = removeSysMonDupRequest(INDICATOR_LIGHT_REQ);
    if(bRet)
    {
        log_warn("%s indicator light request found and remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s indicator light request not in map", __FUNCTION__);
    }

    // query the indicator light style first
    iRet = queryIndicatorLight(style, color, brightness);
    if(0 > iRet)
    {
        log_error("%s query indicator light style failed!!!",__FUNCTION__);
        return iRet;
    }
    else
    {
        log_info("%s light style = %d, light color = %d , light brightness = %d",__FUNCTION__, style, color, brightness);
    }

    if(brightness == light_brightness)
    {
    	log_warn("%s indicator light brightness already at : %d\r\n",__FUNCTION__, brightness);
    	return CTRL_REQ_SUCCESS;
    }

    // set the indicator light brightness according to input param
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = INDICATOR_LIGHT_REQ;
    request.data_i[0] = 1; // set command
    request.data_i[1] = style;
    request.data_i[2] = color;
    request.data_i[3] = light_brightness;
    request.data_i[4] = 1; // get control from mcu

    SYS_MONITOR_CMD_SEND(5);

    indicator_light_style_ = response.data_i[0];
    indicator_light_color_ = response.data_i[1];
    indicator_light_brightness_ = response.data_i[2];

    if(light_brightness != response.data_i[2])
    {
        log_error("%s set indicator light brightness to %d failed!!! , response light brightness = %d",__FUNCTION__, light_brightness , response.data_i[2]);
        return CTRL_REQ_INNER_ERROR;
    }

    log_info("%s set indicator light brightness to %d success... , response light brightness = %d",__FUNCTION__, light_brightness , response.data_i[2]);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

    return CTRL_REQ_SUCCESS;
}

// release the control back to mcu
int McuManager::releaseIndicatorLightControl(void)
{
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);
    // multithread protect lock

    std::unique_lock<std::mutex> lock(indicator_light_mutex_);

    int style, color, brightness;
    int iRet;
    bool bRet;

    bRet = removeSysMonDupRequest(INDICATOR_LIGHT_REQ);
    if(bRet)
    {
        log_warn("%s indicator light request found and remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s indicator light request not in map", __FUNCTION__);
    }

    // set the indicator light brightness according to input param
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = INDICATOR_LIGHT_REQ;
    request.data_i[0] = 1; // set command
    request.data_i[1] = 0; // fill in whatever value for style , since we return the control back to mcu
    request.data_i[2] = 1;
    request.data_i[3] = 0;
    request.data_i[4] = 0; // return control back to mcu

    SYS_MONITOR_CMD_SEND(5);
    
    log_info("%s return the control back to mcu success...", __FUNCTION__);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);
    return CTRL_REQ_SUCCESS;
}





// query control source status
// input parameter control source 1 control source 2
// status = 1 control source is open, status = 0 control source is closed
int McuManager::querySysMonitorControlSourceState(int & control_source_1, int & control_source_2)
{
    std::unique_lock<std::mutex> lock(sys_controlled_source_mutex_);

    bool bRet;
    bRet = removeSysMonDupRequest(CONTROLLED_SOURCE_REQ);
    if(bRet)
    {
        log_warn("%s control source controlled request found and remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s control source controlled request not in map", __FUNCTION__);
    }

    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = CONTROLLED_SOURCE_REQ;
    request.data_i[0] = 0; // query command

    SYS_MONITOR_CMD_SEND(5);

    log_info("%s query controlled source info success... control source 1 : %d , control source 2 : %d", __FUNCTION__, response.data_i[1] & 0x01, response.data_i[1] & 0x02);

    control_source_1 = response.data_i[1] & 0x01;
    control_source_2 = response.data_i[1] & 0x02;

    return CTRL_REQ_SUCCESS;
}

int McuManager::querySysMonitorControlSource(int & control_source_1, int & control_source_2)
{
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = CONTROLLED_SOURCE_REQ;
    request.data_i[0] = 0; // query command

    SYS_MONITOR_CMD_SEND(5);

    log_info("%s query controlled source info success... control source 1 : %d , control source 2 : %d", __FUNCTION__, response.data_i[1] & 0x01, response.data_i[1] & 0x02);

    control_source_1 = response.data_i[1] & 0x01;
    control_source_2 = response.data_i[1] & 0x02;

    return CTRL_REQ_SUCCESS;
}

// set control source 1 status
// status = 1 open control source 1
// status = 0 close control source 1

int McuManager::setSysMonitorControlSource1(int status)
{
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);
    // multithread protection of the controlled voltage source internal state
    std::unique_lock<std::mutex> lock(sys_controlled_source_mutex_);
    // first judge if the light brightness command is in range 
    if(status < 0 || status > 1)
    {
        log_error("set control source 1 input parameter invalid!!!");
        return CTRL_REQ_PARAM_INVALID;
    }

    int control_source_one, control_source_two;
    int iRet;
    bool bRet;

    bRet = removeSysMonDupRequest(CONTROLLED_SOURCE_REQ);
    if(bRet)
    {
        log_warn("%s control source controlled request found and remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s control source controlled request not in map", __FUNCTION__);
    }

    // query the controlled source status info first
    iRet = querySysMonitorControlSource(control_source_one, control_source_two);
    if(0 > iRet)
    {
        log_error("%s query control source status failed!!!",__FUNCTION__);
        return iRet;
    }

    if(control_source_one == status)
    {
    	log_warn("%s system monitor control source 1 already at state : %d", __FUNCTION__, control_source_one);
    	return CTRL_REQ_SUCCESS;
    }

    // set the supplement light style according to input param
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = CONTROLLED_SOURCE_REQ;
    request.data_i[0] = 1; // set command
    request.data_i[1] = status + (control_source_two << 1);

    SYS_MONITOR_CMD_SEND(5);

    sys_controlled_source1_ = response.data_i[0] & 0x01;
    sys_controlled_source2_ = response.data_i[0] & 0x02;

    if(status != (response.data_i[0] & 0x01))
    {
        log_error("%s set controlled voltage source 1 to status : %d failed!!!",__FUNCTION__, status);
        return CTRL_REQ_INNER_ERROR;
    }

    log_info("%s set controlled voltage source 1 to status : %d success...",__FUNCTION__, response.data_i[0] & 0x01);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

    return CTRL_REQ_SUCCESS;

}

// set control source 2 status
// status = 1 open control source 2
// status = 0 close control source 2
int McuManager::setSysMonitorControlSource2(int status)
{
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);

    // multithread protection of the controlled voltage source internal state
    std::unique_lock<std::mutex> lock(sys_controlled_source_mutex_);
    // first judge if the light brightness command is in range 
    if(status < 0 || status > 1)
    {
        log_error("set control source 2 status input parameter invalid!!!");
        return CTRL_REQ_PARAM_INVALID;
    }

    int control_source_one, control_source_two;
    int iRet;
    bool bRet;

    bRet = removeSysMonDupRequest(CONTROLLED_SOURCE_REQ);
    if(bRet)
    {
        log_warn("%s control source controlled request found and remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s control source controlled request not in map",__FUNCTION__);
    }

    // query the supplement light style first
    iRet = querySysMonitorControlSource(control_source_one, control_source_two);
    if(0 > iRet)
    {
        log_error("%s query control source status failed!!!",__FUNCTION__);
        return iRet;
    }

    if(control_source_two == status)
    {
    	log_warn("%s system monitor control source 2 already at state : %d", __FUNCTION__, control_source_two);
    	return CTRL_REQ_SUCCESS;
    }

    // set the supplement light style according to input param
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = CONTROLLED_SOURCE_REQ;
    request.data_i[0] = 1; // set command
    request.data_i[1] = (status << 1) + control_source_one;

    SYS_MONITOR_CMD_SEND(5);

    sys_controlled_source1_ = response.data_i[0] & 0x01;
    sys_controlled_source2_ = response.data_i[0] & 0x02;

    if(status != (response.data_i[0] & 0x02))
    {
        log_error("%s set controlled voltage source 2 to status : %d failed!!!",__FUNCTION__, status);
        return CTRL_REQ_INNER_ERROR;
    }

    log_info("%s set controlled voltage source 2 to status : %d success...",__FUNCTION__, response.data_i[0] & 0x02);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

    return CTRL_REQ_SUCCESS;
}

int McuManager::queryUltraSoundVersion(int & version_info)
{
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = QUERY_ULTRA_VERSION_REQ;

    SYS_MONITOR_CMD_SEND(5);

    log_info("%s query ultra sound version info success... version : %d", __FUNCTION__, response.data_i[0]);

    version_info = response.data_i[0];
    ultra_version_info_ = response.data_i[0];

    return CTRL_REQ_SUCCESS;
}

// process ultra sound message report from sys monitor
// cmd id = 224
void McuManager::processUltraMsg(const tinyros::atris_msgs::CanPkg &msg)
{
    // receive the ultra sound message and publish the message out 
    // TODO ::
    // log_info("%s ultra sound data[0] : %d , ultra sound data[1] : %d , ultra sound data[2] : %d , ultra sound data[3] : %d", __FUNCTION__, msg.data_i[0], msg.data_i[1], msg.data_i[2], msg.data_i[3]);

    // ultra sound message publish out here
    struct timeval ultra_info_recv_this;
    gettimeofday(&ultra_info_recv_this,NULL);
    Json::FastWriter fw;
    Json::Value rbtInfoValue;
    atris_msgs::RobotInfo rbtInfo;
    //int diag_status;

    for(int i = 0; i < 4; i++)
    {
        uint16_t val = msg.data_i[i];
        int idx = i;
        ultra_data[idx] = val;
        //if (ultra_data[idx] == (uint16_t)0xFFFF)
        if (ultra_data[idx] >= 1000) 
        {
            diag_status |= (0x01 << idx);
        } 
        else 
        {
            diag_status &= ~(0x01 << idx);
        }
    }

    rbtInfoValue["robot_info"]["ultrasound"]["error"] = diag_status;

    for (int i=0; i<ULTRA_NUM; i++)
    {
        rbtInfoValue["robot_info"]["ultrasound"]["data"].append(ultra_data[i]);
    }


    if((ultra_info_recv_this.tv_sec - ultra_info_msg_recv_last_) > 5)
    {
        rbtInfo.json = fw.write(rbtInfoValue);
        diag_info_pub_.publish(rbtInfo);
        ultra_info_msg_recv_last_ = ultra_info_recv_this.tv_sec;
    }
}

// cmd id = 214
// cycle report = 300ms
void McuManager::processVoltageInfoMsg(const tinyros::atris_msgs::CanPkg &msg)
{
    // parse voltage monitor information
    static long long recv_sys_voltage_info_last = 0;
    struct timeval sys_voltage_info_recv_this;
    gettimeofday(&sys_voltage_info_recv_this,NULL);

    //log_info("votage source 1 voltage = %d , votage source 2 voltage = %d , votage source 3 voltage = %d , votage source 4 voltage = %d", msg.data_i[0], msg.data_i[1], msg.data_i[2], msg.data_i[3]);

    if((sys_voltage_info_recv_this.tv_sec - recv_sys_voltage_info_last) > 5)
    {
        Json::Value root;
        atris_msgs::RobotInfo rbtInfo_sys_voltage;  
        Json::FastWriter fw;
        root["robot_info"]["system_voltage"]["source1_voltage"] = msg.data_i[0];
        root["robot_info"]["system_voltage"]["source2_voltage"] = msg.data_i[1];
        root["robot_info"]["system_voltage"]["source3_voltage"] = msg.data_i[2];
        root["robot_info"]["system_voltage"]["source4_voltage"] = msg.data_i[3];

        rbtInfo_sys_voltage.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo_sys_voltage);

        recv_sys_voltage_info_last = sys_voltage_info_recv_this.tv_sec;
    }
}

void McuManager::processCurrentInfoMsg(const tinyros::atris_msgs::CanPkg &msg)
{
    // parse voltage source monitor current information
    static long long recv_sys_current_info_last = 0;
    struct timeval sys_current_info_recv_this;
    gettimeofday(&sys_current_info_recv_this,NULL);

    //log_info("votage source 1 current = %d , votage source 2 current = %d , votage source 3 current = %d , votage source 4 current = %d", msg.data_i[0], msg.data_i[1], msg.data_i[2], msg.data_i[3]);

    if((sys_current_info_recv_this.tv_sec - recv_sys_current_info_last) > 5)
    {
        Json::Value root;
        atris_msgs::RobotInfo rbtInfo_sys_voltage;
        Json::FastWriter fw;
        root["robot_info"]["system_current"]["source1_current"] = msg.data_i[0];
        root["robot_info"]["system_current"]["source2_current"] = msg.data_i[1];
        root["robot_info"]["system_current"]["source3_current"] = msg.data_i[2];
        root["robot_info"]["system_current"]["source4_current"] = msg.data_i[3];

        rbtInfo_sys_voltage.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo_sys_voltage);

        recv_sys_current_info_last = sys_current_info_recv_this.tv_sec;
    }
}

// process system monitor temperature and humidity reported message
// cmd id = 226
// report cycle = 300ms

void McuManager::processTempHumiMsg(const tinyros::atris_msgs::CanPkg &msg)
{
    // receive the temperature humidity message and publish the message out 
    // TODO ::

    static long long recv_sys_temp_humi_last = 0;
    struct timeval sys_temp_humi_recv_this;
    gettimeofday(&sys_temp_humi_recv_this,NULL);

    //log_info("%s system temp data : %d , system humi data : %d",__FUNCTION__, msg.data_i[0] , msg.data_i[1]);
    //log_info("%s system NTC temp 1 : %d , system NTC temp 2 : %d , system NTC temp 3 : %d , system NTC temp 4 : %d",__FUNCTION__, msg.data_i[2] , msg.data_i[3] , msg.data_i[4] , msg.data_i[5]);

    if((sys_temp_humi_recv_this.tv_sec - recv_sys_temp_humi_last) > 5)
    {
        Json::Value root;
        atris_msgs::RobotInfo rbtInfo_sys_temp_humi;
        Json::FastWriter fw;
        root["robot_info"]["temp_humi"]["sys_temp"] = msg.data_i[0] * 0.1;
        root["robot_info"]["temp_humi"]["sys_humi"] = (int)(msg.data_i[1] * 0.1);
        root["robot_info"]["temp_humi"]["sys_NTC1"] = msg.data_i[2];
        root["robot_info"]["temp_humi"]["sys_NTC2"] = msg.data_i[3];
        root["robot_info"]["temp_humi"]["sys_NTC3"] = msg.data_i[4];
        root["robot_info"]["temp_humi"]["sys_NTC4"] = msg.data_i[5];

        rbtInfo_sys_temp_humi.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo_sys_temp_humi);

        recv_sys_temp_humi_last = sys_temp_humi_recv_this.tv_sec;
    }

}

void McuManager::sendAdsErrorStatus(int val)
{
    //log_info("%s",__FUNCTION__);
    Json::Value root;
    atris_msgs::RobotInfo rbtInfo_anti_drop_error;
    Json::FastWriter fw;
    root["robot_info"]["ads"]["error"] = val;
    rbtInfo_anti_drop_error.json = fw.write(root);
    diag_info_pub_.publish(rbtInfo_anti_drop_error);
}

void McuManager::checkDeviceError(void)
{
    //struct timeval anti_drop_check_this;
    long long current_time_ms;

    while(1)
    {
        // check ads device communication error
        current_time_ms = getsysmstime();
        if(((current_time_ms - recv_ads_last_) > 3*1000))
        {
            sendAdsErrorStatus(-1);
        }
        else
        {
            sendAdsErrorStatus(0);
        }
        
        // check remote controller message error
        current_time_ms = getsysmstime();
        if(((current_time_ms - remote_control_time_recv_last_) > 3*1000))
        {
            //sendAdsErrorStatus(-1);
            // TODO:
            //log_error("%s remote controller error",__FUNCTION__);
        }
        else
        {
            //sendAdsErrorStatus(0);
        }

        usleep(100*1000);
    }
}

// 
// cmd id = 228
// report cycle = 100ms
void McuManager::processAntiDropMsg(const tinyros::atris_msgs::CanPkg &msg)
{
    //log_info("############## %s anti drop %d\r\n", __FUNCTION__, msg.data_i[0]);
    int anti_drop;
    anti_drop = msg.data_i[0];

    atris_msgs::AntiDrop ad;
    ad.trigged = msg.data_i[0];
    anti_drop_pub_.publish(ad);
    
    struct timeval anti_drop_recv_this;
    gettimeofday(&anti_drop_recv_this, NULL);

    if(((anti_drop_recv_this.tv_sec - recv_anti_drop_last_) > 5) || (anti_drop_status_ != anti_drop))
    {
        Json::Value root;
        atris_msgs::RobotInfo rbtInfo_anti_drop_status;
        Json::FastWriter fw;
        root["robot_info"]["ads"]["status"] = (int)(anti_drop);

        rbtInfo_anti_drop_status.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo_anti_drop_status);

        recv_anti_drop_last_ = anti_drop_recv_this.tv_sec;
        
    }

    anti_drop_status_ = anti_drop;
    recv_ads_last_ = ((long long)anti_drop_recv_this.tv_sec) * 1000 +  anti_drop_recv_this.tv_usec/1000;

}

// query fan error status and speed
// for external use
int McuManager::querySysMonitorFanStatus(int & fan_speed, int & fan1_error_status , int & fan2_error_status)
{
    // multithread protection of the fan status internal state
    std::unique_lock<std::mutex> lock(sys_fan_control_mutex_);

    // query the fan internal error status and speed
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;
    bool bRet;
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);

    bRet = removeSysMonDupRequest(FAN_CONTROL_REQ);
    if(bRet)
    {
        log_warn("%s fan control request found and remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s fan control request not in map",__FUNCTION__);
    }

    request.cmd = FAN_CONTROL_REQ;

    SYS_MONITOR_CMD_SEND(5);

    log_info("%s query fan status success... fan speed : %d , fan error status 1 : %d fan error status 2 : %d", __FUNCTION__, response.data_i[0], response.data_i[1] & 0x01, response.data_i[1] & 0x02);

    fan_speed = response.data_i[0];
    fan1_error_status = response.data_i[1] & 0x01;
    fan2_error_status = response.data_i[1] & 0x02;

    sys_fan_speed_ = response.data_i[0];
    sys_fan1_error_status_ = response.data_i[1] & 0x01;
    sys_fan2_error_status_ = response.data_i[1] & 0x02;

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

    return CTRL_REQ_SUCCESS;
}

// set system monitor fan speed
// we set fan1 and fan2 speed together
int McuManager::setSysMonitorFanSpeed(int speed)
{
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);

    // multithread protection of the fan status internal state
    std::unique_lock<std::mutex> lock(sys_fan_control_mutex_);
    // first judge if speed is in range
    if(speed < 0 || speed > 3)
    {
        log_error("%s set control fan speed input parameter invalid!!!", __FUNCTION__);
        return CTRL_REQ_PARAM_INVALID;
    }

    int fan_speed, fan1_error, fan2_error;
    int iRet;
    bool bRet;

    bRet = removeSysMonDupRequest(FAN_CONTROL_REQ);
    if(bRet)
    {
        log_warn("%s fan control request found and remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s fan control request not in map",__FUNCTION__);
    }

    // set the fan speed according to input param
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = FAN_CONTROL_REQ;
    request.data_i[0] = 1; // set command
    request.data_i[1] = speed;

    SYS_MONITOR_CMD_SEND(5);

    log_info("%s set fan speed response , fan speed : %d , fan1 error status : %d , fan2 error status : %d", __FUNCTION__, response.data_i[0], response.data_i[1] & 0x01, response.data_i[1] & 0x02);

    if(speed != response.data_i[0])
    {
        log_error("%s set fan speed to speed level : %d failed!!!",__FUNCTION__, speed);
        return CTRL_REQ_INNER_ERROR;
    }

    sys_fan_speed_ = response.data_i[0];
    sys_fan1_error_status_ = response.data_i[1] & 0x01;
    sys_fan2_error_status_ = response.data_i[1] & 0x02;

    log_info("%s set fan speed to speed level : %d success...",__FUNCTION__, response.data_i[0]);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

    return CTRL_REQ_SUCCESS;
}
///////////////////////////////////////////////////////////////////////////////
// receive message sent from chassis controller
///////////////////////////////////////////////////////////////////////////////
void McuManager::on_recv_chassis_controller_message(const tinyros::atris_msgs::CanPkg &msg)
{
	// print received response message from chassis controller
    if(MCU_REPORT_MSG_DBG)
    {
        //printCanPkgMsg(msg);
    }

    if(msg.cmd == 0x02 || msg.cmd == 122)
    {
        // do not deal with 0x02 upgrade message response and 122 msg
        return;
    }

    if(msg.cmd == BATTERY_INFO_REPORT)
    {
    	procBatMonInfo(msg);
    }
    else if(CHASSIS_CONTROLLER_NOTIFY_ROBOT_OP_REQ == msg.cmd)
    {
    	chassisNotifyRobotOperation(msg);
    }
    else if(ROBOT_INFORM_CHASSIS_CONTROLLER_OP_RESP == msg.cmd)
    {
        log_info("%s receive mcu shutdown or wake up response",__FUNCTION__); // for debug print
        chassisNotifyOperationResp(msg);
    }
    else if(ROBOT_TO_CHASSIS_RECHARGE_RESP == msg.cmd)
    {
        log_info("---------------- received leave pile request response from chassis controller cmd = %d ------------------", msg.cmd);
    }
    else if(ROBOT_TO_CHASSIS_RECHARGE_IN_POSITION_RESP == msg.cmd)
    {
        log_info("---------------- received robot notify it is in position response from mcu cmd = %d --------------------", msg.cmd);
    }
    else if(CHASSIS_TO_ROBOT_RECHARGE_RESULT_REQ == msg.cmd)
    {
    	chassisNotifyRechargeResult(msg);
    }
    else if(CHASSIS_REPORT_VOLTAGE_INFO == msg.cmd)
    {
    	procChassisVoltageMsg(msg);
    }
    else if(CHASSIS_REPORT_CURRENT_INFO == msg.cmd)
    {
    	procChassisCurrentMsg(msg);
    }
    else if(CHASSIS_REPORT_REMOTE_CONTROLLER_INFO == msg.cmd)
    {
    	procChassisRemoteControlMsg(msg);
    }
    else if(CHASSIS_REPORT_ANTI_COLLISION_INFO == msg.cmd)
    {
    	procAntiCollisionInfo(msg);
    }
    else if(CHASSIS_REPORT_TEMP_HUMI_INFO == msg.cmd)
    {
    	procTempHumiInfo(msg);
    }
    else if(CHASSIS_REPORT_STOP_REASON == msg.cmd)
    {
        procChassisStopReasonInfo(msg);
    }
    else if(QUERY_CHASSIS_VER_RESP == msg.cmd)
    {
        procChassisMcuVersion(msg);
    }
    else if(QUERY_BMS_HW_VER_RESP == msg.cmd)
    {
        procBMSHwVer(msg);
    }
    else if(QUERY_BMS_SW_VER_RESP == msg.cmd)
    {
        procBMSSwVer(msg);
    }
    else if(CHASSIS_TO_ROBOT_SET_TRANSPORT_MODE_RESP == msg.cmd)
    {
        procChassisSetTransportModeResp(msg);
    }
    else if(CHASSIS_TO_ROBOT_NOTIFY_MACHINE_STATUS == msg.cmd)
    {
        procChassisMachineStatus(msg);
    }
    else if(CHASSIS_LOG_FILE_GET_RESP == msg.cmd)
    {
        procChassisUploadLogResp(msg);
    }
    else
    {
    	// here we receive chassis controller request response
    	log_info("request response received from chassis controller , msg id : %d\r\n", msg.cmd);
    	doChassisControllerReqResp(msg);
    }
}

// find the corresponding request from map and signal the condition variable
// request = (cmd)
// response = (cmd id + 1)

void McuManager::doChassisControllerReqResp(const tinyros::atris_msgs::CanPkg &msg)
{
    std::unique_lock<std::mutex> lock(chassis_controller_map_mutex_);

    if (chassis_controller_resp_map_.count(msg.cmd) > 0)
    {
        log_info("%s found corresponding request waited command in map , resp cmd id : %d\r\n", __FUNCTION__, msg.cmd);
        std::unique_lock<std::mutex> resp_lock(chassis_controller_resp_map_[msg.cmd]->mutex);
        chassis_controller_resp_map_[msg.cmd]->resp = msg;
        chassis_controller_resp_map_[msg.cmd]->cond->notify_all();
    }
    else
    {
        log_error("not found corresponding request waited in chassis ctrl response map , id : %d", msg.cmd);
    }
}

// remove duplicate request from map

bool McuManager::removeChassisDupRequest(int msg_id)
{
    std::unique_lock<std::mutex> lock(chassis_controller_map_mutex_);
    if (chassis_controller_resp_map_.count(msg_id + 1) > 0) 
    {
        log_info("%s found corresponding request duplicate message waited in map , request cmd id : %d\r\n", __FUNCTION__, msg_id);
        chassis_controller_resp_map_.erase(msg_id + 1);
        return true;
    }

    log_info("%s ok !!! since no dup request in response map , msg_id:%d", __FUNCTION__, msg_id);
    return false;
}

// print chassis controller report message

void McuManager::printCanPkgMsg(const tinyros::atris_msgs::CanPkg &msg)
{
    log_info("%s cmd id : %d\r\n", __FUNCTION__, msg.cmd);
    log_info("data char \r\n");
    for(int i = 0; i < 16; i++)
    {
        log_info("[%d] th : %c\r\n", i, msg.data_c[i]);
    }

    log_info("\r\n");
    log_info("data int \r\n");

    for(int i = 0; i < 32; i++)
    {
        log_info("[%d] th : %d\r\n", i, msg.data_i[i]);
    }

    log_info("\r\n");
    log_info("data float \r\n");

    for(int i = 0; i < 16; i++)
    {
        log_info("[%d] th : %f", i, msg.data_f[i]);
    }

    log_info("\r\n");
    log_info("data string : %s\r\n",msg.data_s.c_str());
}

// receive voltage info report from chassis controller
// response cmd id = 134

void McuManager::procChassisVoltageMsg(const tinyros::atris_msgs::CanPkg &msg)
{
    static long chassis_voltage_recv_last = 0;
    struct timeval chassis_voltage_recv_this;
    gettimeofday(&chassis_voltage_recv_this,NULL);
    // report to diagnostics module every 5 sec
    #if 0
    log_info("source1_voltage : %d", msg.data_i[0]);
    log_info("source2_voltage : %d", msg.data_i[1]);
    log_info("source3_voltage : %d", msg.data_i[2]);
    log_info("source4_voltage : %d", msg.data_i[3]);
    #endif

    if((chassis_voltage_recv_this.tv_sec - chassis_voltage_recv_last) > 5)
    {
        Json::Value root;
        atris_msgs::RobotInfo rbtInfo_chassis_voltage;  
        Json::FastWriter fw;
        root["robot_info"]["chassis_voltage"]["source1_voltage"] = msg.data_i[0];
        root["robot_info"]["chassis_voltage"]["source2_voltage"] = msg.data_i[1];
        root["robot_info"]["chassis_voltage"]["source3_voltage"] = msg.data_i[2];
        root["robot_info"]["chassis_voltage"]["source4_voltage"] = msg.data_i[3];

        rbtInfo_chassis_voltage.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo_chassis_voltage);

        chassis_voltage_recv_last = chassis_voltage_recv_this.tv_sec;
    }
}

// receive current info report from chassis controller
// response cmd id = 136

void McuManager::procChassisCurrentMsg(const tinyros::atris_msgs::CanPkg &msg)
{
    static long chassis_current_recv_last = 0;
    struct timeval chassis_current_recv_this;
    gettimeofday(&chassis_current_recv_this,NULL);

    // report to diagnostics module every 5 sec
    #if 0
    log_info("source1_current : %d", msg.data_i[0]);
    log_info("source2_current : %d", msg.data_i[1]);
    log_info("source3_current : %d", msg.data_i[2]);
    log_info("source4_current : %d", msg.data_i[3]);
    #endif

    if((chassis_current_recv_this.tv_sec - chassis_current_recv_last) > 5)
    {
        Json::Value root;
        atris_msgs::RobotInfo rbtInfo_chassis_current;  
        Json::FastWriter fw;
        root["robot_info"]["chassis_current"]["source1_current"] = msg.data_i[0];
        root["robot_info"]["chassis_current"]["source2_current"] = msg.data_i[1];
        root["robot_info"]["chassis_current"]["source3_current"] = msg.data_i[2];
        root["robot_info"]["chassis_current"]["source4_current"] = msg.data_i[3];

        rbtInfo_chassis_current.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo_chassis_current);
        chassis_current_recv_last = chassis_current_recv_this.tv_sec;
    }
}

// receive remote control reported channel info
// twist

void McuManager::procChassisRemoteControlMsg(const tinyros::atris_msgs::CanPkg &msg)
{
    //int flag;
    int ch_1,ch_2,ch_3,ch_4,ch_5,ch_6,ch_7,ch_8,ch_9,ch_10,ch_11,ch_12,ch_13,ch_14,ch_15,ch_16;
    //int remote_status;

    ch_flag_ = msg.data_i[0];
    ch_1 = msg.data_i[1];
    ch_2 = msg.data_i[2];
    ch_3 = msg.data_i[3];
    ch_4 = msg.data_i[4];
    ch_5 = msg.data_i[5];
    ch_6 = msg.data_i[6];
    ch_7 = msg.data_i[7];
    ch_8 = msg.data_i[8];
    ch_9 = msg.data_i[9];
    ch_10 = msg.data_i[10];
    ch_11 = msg.data_i[11];
    ch_12 = msg.data_i[12];
    ch_13 = msg.data_i[13];
    ch_14 = msg.data_i[14];
    ch_15 = msg.data_i[15];
    ch_16 = msg.data_i[16];

    //long long remote_control_time_this;
    //remote_control_time_this = getsysmstime();
    //log_info("%s function get called interval = %llu\r\n", __FUNCTION__, remote_control_time_this - remote_control_time_recv_last_);
    remote_control_time_recv_last_ = getsysmstime();

    #if 0
    log_info("%s off_line status:%d,ch1:%d,ch2:%d,ch3:%d,ch4:%d,ch5:%d,ch6:%d,ch7:%d,ch8:%d,ch9:%d,ch10:%d,ch11:%d,ch12:%d,ch13:%d,ch14:%d,ch15:%d,ch16:%d,channel flag:%.2x",__FUNCTION__,remote_status_,
               ch_1,
               ch_2,
               ch_3,
               ch_4,
               ch_5,
               ch_6,
               ch_7,
               ch_8,
               ch_9,
               ch_10,
               ch_11,
               ch_12,
               ch_13,
               ch_14,
               ch_15,
               ch_16,
               ch_flag_);
    #endif
    //log_info("%s off_line status:%d,ch6:%d,ch8:%d,channel flag:%.2x",__FUNCTION__,remote_status_,ch_6, ch_8, ch_flag_);

    if(ch_flag_ & 0xc)
    {
        if(remote_status_ != DF_REMOTE_OFF_LINE)
        {
            remote_status_ = DF_REMOTE_OFF_LINE;
            log_info("==================================================================");
            log_info("=======================%s Remote Controller power off =============", __FUNCTION__);
            log_info("==================================================================");
            log_info("report remote controller power off message to shadow server");
            setVraVal(-1);
            vra_filter_out_val_ = 0;
            setVrbVal(-1);
            vrb_filter_out_val_ = 0;

            notifyRemoteControllerInControl(0);

            atris_msgs::AisoundTTS tts_msg;
            tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
            tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_TELECONTROL_POW_OFF);
            aisound_tts_pub_.publish(tts_msg);
        }

        return;
    }
    else if(remote_status_ == DF_REMOTE_OFF_LINE)
    {
        if(remote_status_ != DF_REMOTE_ON)
        {
            log_info("==================================================================");
            log_info("=====================%s Remote Controller power on ================", __FUNCTION__);
            log_info("==================================================================");
            log_info("report remote controller power on message to shadow server");
            setVraVal(-1);
            vra_filter_out_val_ = 0;
            setVrbVal(-1);
            vrb_filter_out_val_ = 0;
            remote_status_ = DF_REMOTE_ON;
            notifyRemoteControllerInControl(1);

            atris_msgs::AisoundTTS tts_msg;
            tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
            tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_TELECONTROL_POW_ON);
            aisound_tts_pub_.publish(tts_msg);
        }
    }

    if((ch_14 > (REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) && (remote_status_ == DF_REMOTE_ON))
    {
        remote_status_ = DF_REMOTE_ON_LINE;
        log_info("=============%s set remote controller online",__FUNCTION__);
        log_info("=============%s Remote Controller lock after power on", __FUNCTION__);

    }

    if((ch_14 < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) && ((remote_status_ == DF_REMOTE_ON_LINE) || (remote_status_ == DF_REMOTE_ROCKER_NOT_ON_CENTER)))
    {
        if( (abs(ch_1 -REMOTE_CTRL_SBUS_CENTER) < REMOTE_CTRL_ABNOR_VALUL) && 
            (abs(ch_2 -REMOTE_CTRL_SBUS_CENTER) < REMOTE_CTRL_ABNOR_VALUL) && 
            (abs(ch_3 -REMOTE_CTRL_SBUS_CENTER) < REMOTE_CTRL_ABNOR_VALUL) && 
            (abs(ch_4 -REMOTE_CTRL_SBUS_CENTER) < REMOTE_CTRL_ABNOR_VALUL))
        {
            remote_status_ = DF_REMOTE_CONTROL;
            log_info("==================%s Remote Controller set to remote control mode", __FUNCTION__);
            log_info("==================%s Remote Controller unlock", __FUNCTION__);
            setVraVal(-1);
            vra_filter_out_val_ = 0;
            setVrbVal(-1);
            vrb_filter_out_val_ = 0;
        }
        else if(remote_status_ !=  DF_REMOTE_ROCKER_NOT_ON_CENTER)
        {
            remote_status_ = DF_REMOTE_ROCKER_NOT_ON_CENTER;
            //send_twish(REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_MAX_VALUE);
            log_info("=================%s Please center the rocker of the remote controller before control the robot!", __FUNCTION__);
 
        }

    }
    else if((ch_14 > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) && (remote_status_ == DF_REMOTE_CONTROL))
    {
        remote_status_ = DF_REMOTE_ON_LINE;
        //send_twish(REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_MAX_VALUE);
        log_info("%s can not control but online", __FUNCTION__);
        log_info("==================%s Remote Controller lock!!!\r\n",__FUNCTION__);
        setVraVal(-1);
        vra_filter_out_val_ = 0;
        setVrbVal(-1);
        vrb_filter_out_val_ = 0;
    }


    //if(remote_status_ == DF_REMOTE_CONTROL)
    // we can control it anyway right now
    // do not need to unlock it in remote control mode
    if(remote_status_ == DF_REMOTE_CONTROL)
    {
        // this two parts just for test
        // control ptz up and down
        #if 0
        if((ch_9 < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) && (ptz_up_down_state_ != PTZ_VERTICAL_MOVE_UP))
        {
            ptz_up_down_state_ = PTZ_VERTICAL_MOVE_UP;
            reqCtrlPtzMove(0);
            log_info("===================%s remote control ptz vertical move up , ptz_up_down_state = %d =============================",__FUNCTION__, ptz_up_down_state_);
        }
        else if(((ch_9 >= ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL))&& (ch_9 <= ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL))) && (ptz_up_down_state_ != PTZ_VERTICAL_MOVE_IDLE))
        {
            ptz_up_down_state_ = PTZ_VERTICAL_MOVE_IDLE;
            log_info("===================%s remote control ptz vertical move idle , ptz_up_down_state = %d =============================",__FUNCTION__, ptz_up_down_state_);
        }
        else if((ch_9 > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) && (ptz_up_down_state_ != PTZ_VERTICAL_MOVE_DOWN))
        {
            ptz_up_down_state_ = PTZ_VERTICAL_MOVE_DOWN;
            reqCtrlPtzMove(1);
            log_info("===================%s remote control ptz vertical move down , ptz_up_down_state = %d =============================",__FUNCTION__, ptz_up_down_state_);
        }

        // control ptz left and right

        if((ch_11 < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) && (ptz_left_right_state_ != PTZ_HORIZONTAL_MOVE_LEFT))
        {
            ptz_left_right_state_ = PTZ_HORIZONTAL_MOVE_LEFT;
            reqCtrlPtzMove(2);
            log_info("===================%s remote control ptz horizontal move left , ptz_left_right_state = %d =============================",__FUNCTION__, ptz_left_right_state_);
        }
        else if(((ch_11 >= ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL))&& (ch_11 <= ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL))) && (ptz_left_right_state_ != PTZ_HORIZONTAL_MOVE_IDLE))
        {
            ptz_left_right_state_ = PTZ_HORIZONTAL_MOVE_IDLE;
            log_info("===================%s remote control ptz horizontal move idle , ptz_left_right_state = %d =============================",__FUNCTION__, ptz_left_right_state_);
        }
        else if((ch_11 > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) && (ptz_left_right_state_ != PTZ_HORIZONTAL_MOVE_RIGHT))
        {
            ptz_left_right_state_ = PTZ_HORIZONTAL_MOVE_RIGHT;
            reqCtrlPtzMove(3);
            log_info("===================%s remote control ptz horizontal move right , ptz_left_right_state = %d =============================",__FUNCTION__, ptz_left_right_state_);
        }
        #endif

        remoteControlChassisTransportMode(ch_16);

        remoteCtrlChassisUnlock(ch_15); // ptz unlock

        remoteCtrlPtzLight(ch_5); // control ptz light

        remoteCtrlPtzBrush(ch_13); // control ptz brush

        //remoteControlPtzUpDown(ch_6); // vra knob control

        //remoteControlPtzLeftRight(ch_8); // vrb knob control
	remoteControlPtzVertical(ch_10);

        remoteControlPtzHorizontal(ch_11); // second method to control ptz move horizontally
    }

}

// filter out vra knob recv value
int McuManager::filterVrANoise(int chan_val)
{
    if(abs(chan_val - getVraVal()) < KNOB_NOISE_TOLERANCE_MAX)
    {
        log_warn("chan val : %d , vra value last : %d", chan_val, getVraVal());
        setVraVal(chan_val);
        return -1;
    }
    else
    {
        //log_info("channel info value ok chan_val = %d", chan_val);
    }

    return 0;
}

// filter out vrb knob recv value
int McuManager::filterVrBNoise(int chan_val)
{
    if(abs(chan_val - getVrbVal()) < KNOB_NOISE_TOLERANCE_MAX)
    {
        log_warn("chan val : %d , vrb value last : %d", chan_val, getVrbVal());
        setVrbVal(chan_val);
        return -1;
    }
    else
    {
        //log_info("channel info value ok chan_val = %d", chan_val);
    }

    return 0;
}

// clamping the input value between min and max
int McuManager::chanValClamp(int val, int min, int max)
{
    if(val < min)
    {
        return min;
    }

    if(val > max)
    {
        return max;
    }

    return val;
}

bool McuManager::waitInputVraStable(int channel_value)
{
    //log_info("%s channel value = %d , vra last value = %d, vra stable count = %d",__FUNCTION__, channel_value, getVraVal(), vra_stable_count_);
    if(abs(channel_value - getVraVal()) < KNOB_NOISE_TOLERANCE_MAX)
    {
        //log_warn("%s chan val : %d , vra value last : %d vra_stable_count_ : %d ", __FUNCTION__, channel_value, getVraVal(), vra_stable_count_);
        if(vra_stable_state_ == false && vra_stable_count_ != KNOB_WAIT_STABLE_CNT)
        {
            vra_stable_count_++;
        }
    }
    else
    {
        // clear the vra stable count
        if(vra_filter_out_val_ == 1) // only if the first time value is filtered out
        {
            vra_stable_state_ = false;
            vra_stable_count_ = 0;
            vra_stable_send_once_ = 0;
        }
    }

    setVraVal(channel_value);

    // filter out the first abnormal value
    if(vra_filter_out_val_ == 0)
    {
        log_info("%s vra filter out the first time value",__FUNCTION__);
        vra_filter_out_val_ = 1;
        return false;
    }

    if(vra_stable_state_ == false && vra_stable_count_ == KNOB_WAIT_STABLE_CNT)
    {
        vra_stable_state_ = true;
        if(!vra_stable_send_once_)
        {
            vra_stable_send_once_ = 1;
            log_info("%s *************************",__FUNCTION__);
            log_info("%s vra wait stable ok", __FUNCTION__);
            log_info("%s *************************",__FUNCTION__);
            return true;
        }
        else
        {
            return false;
        }
    }

    return false;
}

bool McuManager::waitInputVrbStable(int channel_value)
{
    //log_info("%s channel value = %d , vrb last value = %d, vrb stable count = %d",__FUNCTION__, channel_value, getVrbVal(), vrb_stable_count_);
    if(abs(channel_value - getVrbVal()) < KNOB_NOISE_TOLERANCE_MAX)
    {
        //log_warn("%s chan val : %d , vrb value last : %d vrb_stable_count_ : %d ", __FUNCTION__, channel_value, getVrbVal(), vrb_stable_count_);
        if(vrb_stable_state_ == false && vrb_stable_count_ != KNOB_WAIT_STABLE_CNT)
        {
            vrb_stable_count_++;
        }
    }
    else
    {
        // clear the vrb stable count
        if(vrb_filter_out_val_ == 1) // only if the first time value is filtered out
        {
            vrb_stable_state_ = false;
            vrb_stable_count_ = 0;
            vrb_stable_send_once_ = 0;
        }
    }

    setVrbVal(channel_value);

    // filter out the first abnormal value
    if(vrb_filter_out_val_ == 0)
    {
        log_info("%s vrb filter out the first time value",__FUNCTION__);
        vrb_filter_out_val_ = 1;
        return false;
    }

    if(vrb_stable_state_ == false && vrb_stable_count_ == KNOB_WAIT_STABLE_CNT)
    {
        vrb_stable_state_ = true;
        if(!vrb_stable_send_once_)
        {
            vrb_stable_send_once_ = 1;
            log_info("%s *************************",__FUNCTION__);
            log_info("%s vrb wait stable ok", __FUNCTION__);
            log_info("%s *************************",__FUNCTION__);
            return true;
        }
        else
        {
            return false;
        }
    }

    return false;
}

// remote control ptz supplement light
void McuManager::remoteCtrlPtzLight(int chan_val)
{
    if((chan_val < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  (supplement_light_state_!= PTZ_SUPPLEMENT_LIGHT_OFF))
    {
        log_info("==============%s,remote control ptz supplement light off ====================",__FUNCTION__);
        reqCtrlPtzLight(1);
        supplement_light_state_ = PTZ_SUPPLEMENT_LIGHT_OFF;
    }
    else if((chan_val > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  (supplement_light_state_ != PTZ_SUPPLEMENT_LIGHT_ON))
    {
        log_info("===============%s,remote control ptz supplement light on ==================",__FUNCTION__);
        reqCtrlPtzLight(0);
        supplement_light_state_ = PTZ_SUPPLEMENT_LIGHT_ON;
    }
}

void McuManager::reqCtrlPtzLight(int switch_val)
{
    log_info("%s called, switch val = %d",__FUNCTION__, switch_val);
    Json::FastWriter jwriter;
    Json::Value root;
    atris_msgs::SignalMessage ptz_msg;
    std::string ptz_msg_content = ""; // for debug purpose
    tinyros::Time now = tinyros::Time::now();
    std::stringstream uid; 
    uid << ((uint64_t) (now.toSec() * 1000000000ull));

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["title"] = "request_switch_ptz_light";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["id"] = Json::Value(uid.str());
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000); // in ms
    root["content"]["switch"] = Json::Value(switch_val);
    ptz_msg_content = jwriter.write(root);
    log_info("%s remote control request ptz control supplement light %s",__FUNCTION__, ptz_msg_content.c_str());
    ptz_msg.title = "request_switch_ptz_light";
    ptz_msg.msg = jwriter.write(root);
    ptz_msg.msgID = uid.str();
    ptz_msg.timestamp = root["content"]["timestamp"].asInt64();
    ptz_ctrl_pub_.publish(ptz_msg);
}

// remote unlock chassis(using software method)
void McuManager::remoteCtrlChassisUnlock(int chan_val)
{
    if((chan_val < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  (brake_state_!= REMOTE_ENABLE_BRAKE_OFF))
    {
        log_info("==============%s,remote control software chassis unlock ====================",__FUNCTION__);
        //reqCtrlPtzBrush(0);
        sendChassisUnlockNonBlock(0);
        brake_state_ = REMOTE_ENABLE_BRAKE_OFF;
    }
    else if((chan_val > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  (brake_state_ != REMOTE_ENABLE_BRAKE_NULL))
    {
        log_info("===============%s,remote control chassis unlock null ==================",__FUNCTION__);
        brake_state_ = REMOTE_ENABLE_BRAKE_NULL;
    }
}

// remote control ptz brush on
void McuManager::remoteCtrlPtzBrush(int chan_val)
{
    if((chan_val < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  (ptz_brush_state_!= PTZ_BRUSH_STATE_ON))
    {
        log_info("==============%s,remote control ptz brush on ====================",__FUNCTION__);
        reqCtrlPtzBrush(0);
        ptz_brush_state_ = PTZ_BRUSH_STATE_ON;
    }
    else if((chan_val > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  (ptz_brush_state_ != PTZ_BRUSH_STATE_OFF))
    {
        log_info("===============%s,remote control ptz brush off ==================",__FUNCTION__);
        reqCtrlPtzBrush(1);
        ptz_brush_state_ = PTZ_BRUSH_STATE_OFF;
    }

}

void McuManager::reqCtrlPtzBrush(int switch_val)
{
    log_info("%s called, switch val = %d",__FUNCTION__, switch_val);
    Json::FastWriter jwriter;
    Json::Value root;
    atris_msgs::SignalMessage ptz_msg;
    std::string ptz_msg_content = ""; // for debug purpose
    tinyros::Time now = tinyros::Time::now();
    std::stringstream uid; 
    uid << ((uint64_t) (now.toSec() * 1000000000ull));

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["title"] = "request_switch_ptz_wiper";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["id"] = Json::Value(uid.str());
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000); // in ms
    root["content"]["switch"] = Json::Value(switch_val);
    ptz_msg_content = jwriter.write(root);
    log_info("%s remote control request ptz control wiper %s",__FUNCTION__, ptz_msg_content.c_str());
    ptz_msg.title = "request_switch_ptz_wiper";
    ptz_msg.msg = jwriter.write(root);
    ptz_msg.msgID = uid.str();
    ptz_msg.timestamp = root["content"]["timestamp"].asInt64();
    ptz_ctrl_pub_.publish(ptz_msg);
    
}

void McuManager::remoteControlPtzUpDown(int chan_val)
{
    // first filter out the jitter value
    int iRet;
    int vertical_ctrl_value;
    int vertical_ctrl_temp;
    int stable_ok;
    #if 0
    iRet = filterVrANoise(chan_val);
    if(iRet < 0)
    {
        log_info("%s remote control vra filter out same and jitter noise value", __FUNCTION__);
        return;
    }
    #endif

    stable_ok = waitInputVraStable(chan_val);
    if(!stable_ok)
    {
        // not sending the control command
        //log_info("%s wait input stable not ok",__FUNCTION__);
        return;
    }
    else
    {
        //log_info("%s wait stable ok",__FUNCTION__);
    }

    vertical_ctrl_temp = (int)((chan_val - REMOTE_KNOB_MIN_VALUE)/PTZ_CONTROL_VERTICAL_FACTOR);
    vertical_ctrl_value = chanValClamp(vertical_ctrl_temp, 0, 180); // clamping the value between 0 and 180
    vertical_ctrl_value -= 90; // offset the value between -90 and 90

    // here we publish the message out
    // TODO jinzhongxi:
    log_info("%s verical value calculated = %d", __FUNCTION__, vertical_ctrl_value);
    log_info("%s publish the message out");
    reqCtrlPtzMoveVerticalAngle(vertical_ctrl_value);
    //setVraVal(chan_val);
}

void McuManager::remoteControlPtzLeftRight(int chan_val)
{
    // first filter out the jitter value
    int iRet;
    int horizontal_ctrl_value;
    int horizontal_ctrl_temp;
    int stable_ok;
    #if 0
    iRet = filterVrANoise(chan_val);
    if(iRet < 0)
    {
        log_info("%s remote control vra filter out same and jitter noise value", __FUNCTION__);
        return;
    }
    #endif

    stable_ok = waitInputVrbStable(chan_val);
    if(!stable_ok)
    {
        // not sending the control command
        //log_info("%s wait input stable not ok",__FUNCTION__);
        return;
    }
    else
    {
        //log_info("%s wait stable ok",__FUNCTION__);
    }

    horizontal_ctrl_temp = (int)((chan_val - REMOTE_KNOB_MIN_VALUE)/PTZ_CONTROL_HORIZONTAL_FACTOR);
    horizontal_ctrl_value = chanValClamp(horizontal_ctrl_temp, 0, 360); // clamping the value between 0 and 180
    
    // here we publish the message out
    // TODO jinzhongxi:
    log_info("%s horizontal value calculated = %d", __FUNCTION__, horizontal_ctrl_value);
    log_info("%s publish the message out");
    reqCtrlPtzMoveHorizontalAngle(horizontal_ctrl_value);
    
}

void McuManager::remoteControlPtzVertical(int chan_val)
{

    if((chan_val < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) && (ptz_up_down_state_ != PTZ_VERTICAL_MOVE_UP))
    {
        // control horizontal rotating left
        ptz_up_down_state_ = PTZ_VERTICAL_MOVE_UP;
        reqCtrlPtzMoveDirection(0);
        log_info("===================%s remote control ptz vertical move up , ptz_up_down_state = %d =============================", __FUNCTION__, ptz_up_down_state_);
    }
    else if(((chan_val >= ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL))&& (chan_val <= ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL))) && (ptz_up_down_state_ != PTZ_VERTICAL_MOVE_IDLE))
    {
        // control horizontal rotating stop
        ptz_up_down_state_ = PTZ_VERTICAL_MOVE_IDLE;
        reqCtrlPtzMoveStop();
        log_info("===================%s remote control ptz vertical move idle , ptz_up_down_state = %d =============================", __FUNCTION__, ptz_up_down_state_);
    }
    else if((chan_val > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) && (ptz_up_down_state_ != PTZ_VERTICAL_MOVE_DOWN))
    {
        // control horizontal rotating right
        ptz_up_down_state_ = PTZ_VERTICAL_MOVE_DOWN;
        reqCtrlPtzMoveDirection(1);
        log_info("===================%s remote control ptz vertical move down , ptz_up_down_state = %d =============================", __FUNCTION__, ptz_up_down_state_);
    }
}

// we use channel 9 three level switch to control the ptz rotate horizontally
void McuManager::remoteControlPtzHorizontal(int chan_val)
{
    if((chan_val < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) && (ptz_left_right_state_ != PTZ_HORIZONTAL_MOVE_LEFT))
    {
        // control horizontal rotating left
        ptz_left_right_state_ = PTZ_HORIZONTAL_MOVE_LEFT;
        reqCtrlPtzMoveDirection(2);
        log_info("===================%s remote control ptz horizontal move left , ptz_left_right_state = %d =============================", __FUNCTION__, ptz_left_right_state_);
    }
    else if(((chan_val >= ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL))&& (chan_val <= ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL))) && (ptz_left_right_state_ != PTZ_HORIZONTAL_MOVE_IDLE))
    {
        // control horizontal rotating stop
        ptz_left_right_state_ = PTZ_HORIZONTAL_MOVE_IDLE;
        reqCtrlPtzMoveStop();
        log_info("===================%s remote control ptz horizontal move idle , ptz_left_right_state = %d =============================", __FUNCTION__, ptz_left_right_state_);
    }
    else if((chan_val > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) && (ptz_left_right_state_ != PTZ_HORIZONTAL_MOVE_RIGHT))
    {
        // control horizontal rotating right
        ptz_left_right_state_ = PTZ_HORIZONTAL_MOVE_RIGHT;
        reqCtrlPtzMoveDirection(3);
        log_info("===================%s remote control ptz horizontal move right , ptz_left_right_state = %d =============================", __FUNCTION__, ptz_left_right_state_);
    }
}

// remote control set or unset robot transport mode
void McuManager::remoteControlChassisTransportMode(int chan_val)
{
    int iRet;
    ros::Time curTime = ros::Time::now();

    if(chan_val > (REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL))
    {
        if(!swh_start_to_count_)
        {
            swh_start_to_count_ = true;
            swh_stable_time_last_ = curTime;
        }

        if(swh_start_to_count_ && ((curTime.toSec() - swh_stable_time_last_.toSec()) > 5))
        {
            if(swh_stable_send_once_ == 0)
            {
                swh_stable_send_once_ = 1;
                log_info("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
                log_info("&&&&&& send set chassis in transport mode &&&&&&&");
                log_info("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
                iRet = setChassisTransportMode(1);
                if(iRet < 0)
                {
                    log_error("%s send set chassis in transport mode failde!!! , iRet = %d",__FUNCTION__, iRet);
                }
                else
                {
                    log_info("%s send set chassis in transport mode success...",__FUNCTION__);
                }
            }
        }
    }
    else
    {
        swh_stable_time_last_ = curTime; // clear the time counter
        swh_stable_send_once_ = 0;
        swh_start_to_count_ = false;
    }
}



void McuManager::reqCtrlPtzMoveDirection(int direction)
{
    log_info("%s called",__FUNCTION__);

    Json::FastWriter jwriter;
    Json::Value root;
    atris_msgs::SignalMessage ptz_msg;
    std::string ptz_msg_content = ""; // for debug purpose
    tinyros::Time now = tinyros::Time::now();
    std::stringstream uid; 
    uid << ((uint64_t) (now.toSec() * 1000000000ull));


    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["title"] = "request_ptz_move_control";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["id"] = Json::Value(uid.str());
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000); // in ms
    root["content"]["direction"] = Json::Value(direction);
    ptz_msg_content = jwriter.write(root);
    log_info("%s remote control request ptz horizontal control %s",__FUNCTION__, ptz_msg_content.c_str());
    ptz_msg.title = "request_ptz_move_control";
    ptz_msg.msg = jwriter.write(root);
    ptz_msg.msgID = uid.str();
    ptz_msg.timestamp = root["content"]["timestamp"].asInt64();
    ptz_ctrl_pub_.publish(ptz_msg);
}

void McuManager::reqCtrlPtzMoveStop(void)
{
    log_info("%s called",__FUNCTION__);

    Json::FastWriter jwriter;
    Json::Value root;
    atris_msgs::SignalMessage ptz_msg;
    std::string ptz_msg_content = ""; // for debug purpose
    tinyros::Time now = tinyros::Time::now();
    std::stringstream uid; 
    uid << ((uint64_t) (now.toSec() * 1000000000ull));

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["title"] = "request_ptz_stop_control";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["id"] = Json::Value(uid.str());
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000); // in ms
    ptz_msg_content = jwriter.write(root);
    log_info("%s remote control request ptz horizontal control stop %s",__FUNCTION__, ptz_msg_content.c_str());
    ptz_msg.title = "request_ptz_stop_control";
    ptz_msg.msg = jwriter.write(root);
    ptz_msg.msgID = uid.str();
    ptz_msg.timestamp = root["content"]["timestamp"].asInt64();
    ptz_ctrl_pub_.publish(ptz_msg);
}

void McuManager::reqCtrlPtzMoveVerticalAngle(int angle)
{
    log_info("%s called , angle = %d",__FUNCTION__, angle);
    Json::FastWriter jwriter;
    Json::Value root;
    atris_msgs::SignalMessage ptz_msg;
    std::string ptz_msg_content = ""; // for debug purpose
    tinyros::Time now = tinyros::Time::now();
    std::stringstream uid; 
    uid << ((uint64_t) (now.toSec() * 1000000000ull));

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["title"] = "request_ptz_hv_angle";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["id"] = Json::Value(uid.str());
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000); // in ms
    root["content"]["direction"] = Json::Value(1);
    root["content"]["angle"] = Json::Value(angle*1.0);
    ptz_msg_content = jwriter.write(root);
    log_info("%s remote control request ptz control %s",__FUNCTION__, ptz_msg_content.c_str());
    ptz_msg.title = "request_ptz_hv_angle";
    ptz_msg.msg = jwriter.write(root);
    ptz_msg.msgID = uid.str();
    ptz_msg.timestamp = root["content"]["timestamp"].asInt64();
    ptz_ctrl_pub_.publish(ptz_msg);
}

void McuManager::reqCtrlPtzMoveHorizontalAngle(int angle)
{
    log_info("%s called , angle = %d",__FUNCTION__, angle);
    Json::FastWriter jwriter;
    Json::Value root;
    atris_msgs::SignalMessage ptz_msg;
    std::string ptz_msg_content = ""; // for debug purpose
    tinyros::Time now = tinyros::Time::now();
    std::stringstream uid; 
    uid << ((uint64_t) (now.toSec() * 1000000000ull));

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["title"] = "request_ptz_hv_angle";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["id"] = Json::Value(uid.str());
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000); // in ms
    root["content"]["direction"] = Json::Value(0);
    root["content"]["angle"] = Json::Value(angle*1.0);
    ptz_msg_content = jwriter.write(root);
    log_info("%s remote control request ptz control %s",__FUNCTION__, ptz_msg_content.c_str());
    ptz_msg.title = "request_ptz_hv_angle";
    ptz_msg.msg = jwriter.write(root);
    ptz_msg.msgID = uid.str();
    ptz_msg.timestamp = root["content"]["timestamp"].asInt64();
    ptz_ctrl_pub_.publish(ptz_msg);
}

// use remote controller to request ptz to move in direction
// 0 up 1 down 2 left 3 right
void McuManager::reqCtrlPtzMove(int direction)
{
    log_info("%s called",__FUNCTION__);
    Json::FastWriter jwriter;
    Json::Value root;
    atris_msgs::SignalMessage ptz_msg;
    std::string ptz_msg_content = ""; // for debug purpose
    tinyros::Time now = tinyros::Time::now();
    std::stringstream uid; 
    uid << ((uint64_t) (now.toSec() * 1000000000ull));

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["title"] = "request_ptz_move_control";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["id"] = Json::Value(uid.str());
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000); // in ms
    root["content"]["direction"] = Json::Value(direction);
    ptz_msg_content = jwriter.write(root);
    log_info("%s remote control request ptz control %s",__FUNCTION__, ptz_msg_content.c_str());
    ptz_msg.title = "request_ptz_move_control";
    ptz_msg.msg = jwriter.write(root);
    ptz_msg.msgID = uid.str();
    ptz_msg.timestamp = root["content"]["timestamp"].asInt64();
    ptz_ctrl_pub_.publish(ptz_msg);
}
// notify in control of the remote controller to shadow server
void McuManager::notifyRemoteControllerInControl(uint8_t ctrl_switch)
{
    log_info("%s",__FUNCTION__);
    atris_msgs::RobotRunMode robot_run_mode_msg;
    robot_run_mode_msg.robot_mode = 4;
    robot_run_mode_msg.status = ctrl_switch;
    robot_run_mode_pub_.publish(robot_run_mode_msg);

}

// receive collision bar info from chassis controller
// cmd id = 144
// cycle = 200ms

void McuManager::procAntiCollisionInfo(const tinyros::atris_msgs::CanPkg &msg)
{
    //log_info("%s recv cmd id : %d",__FUNCTION__, msg.cmd);
    //log_info("%s recv anti collision info message",__FUNCTION__);
    int emergency_button_status;
    int collision_bumper_status;
    static long chassis_collision_recv_last = 0;
    struct timeval chassis_collision_recv_this;
    gettimeofday(&chassis_collision_recv_this,NULL);

    // 0x00 means the button is not pressed
    // 0x01 means the button is pressed
    emergency_button_status = msg.data_i[0];
    //log_info("emergency button status : %d", emergency_button_status);

    // 0x00 means the anti collision bar is not triggered 
    // 0x01 means the anti collision bar is triggered 
    // 0x02 means the anti collision bar is vacant

    collision_bumper_status = msg.data_i[1];
    //log_info("collision bar status : %d", collision_bumper_status);
    // publish the bumper status message every 5 secs
    //log_info("%s - 1111, button_status:%d, bumper_status:%d", __FUNCTION__, msg.data_i[0], msg.data_i[1]);
    if((emergency_button_status_ != emergency_button_status)||(collision_bumper_status_ != collision_bumper_status)||(chassis_collision_recv_this.tv_sec - chassis_collision_recv_last) > 5)
    {
        //log_warn("%s recv change of emergency button info or interval > 5s : emergency button = %d , collision bumper status = %d", __FUNCTION__, emergency_button_status, collision_bumper_status);
        //log_warn("%s time interval = %ld secs",__FUNCTION__, chassis_collision_recv_this.tv_sec - chassis_collision_recv_last);
        Json::Value root;
        atris_msgs::RobotInfo rbtInfo_stop_status;
        Json::FastWriter fw;
        root["robot_info"]["brake"]["button"] = emergency_button_status;
        root["robot_info"]["brake"]["front_bumper"] = collision_bumper_status;

        rbtInfo_stop_status.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo_stop_status); // dont publish right now
        chassis_collision_recv_last = chassis_collision_recv_this.tv_sec;
        emergency_button_status_ = emergency_button_status;
        collision_bumper_status_ = collision_bumper_status;

        //log_info("%s - 2222, button_status:%d, bumper_status:%d", __FUNCTION__, msg.data_i[0], msg.data_i[1]);
    }
}

// receive chassis mcu version
// cmd id = 192
void McuManager::procChassisMcuVersion(const tinyros::atris_msgs::CanPkg &msg)
{
    chassis_core_hw_version_ = msg.data_i[0];
    chassis_base_hw_version_ = msg.data_i[1];
    chassis_sw_version_  = msg.data_s;

    log_info("%s recv chassis sw version : %s, core hw version : %d , base hw version :%d",__FUNCTION__, chassis_sw_version_.c_str(), chassis_core_hw_version_, chassis_base_hw_version_);

    isChassisMcuVerReceived = true;
}

// receive monitor mcu version
// cmd id = 292

void McuManager::processMonitorMcuVersion(const tinyros::atris_msgs::CanPkg &msg)
{
    monitor_core_hw_version_ = msg.data_i[0];
    monitor_base_hw_version_ = msg.data_i[1];
    monitor_sw_version_  = msg.data_s;

    log_info("%s recv monitor sw version : %s, core hw version : %d , base hw version :%d",__FUNCTION__, monitor_sw_version_.c_str(), monitor_core_hw_version_, monitor_base_hw_version_);

    isMonitorMcuVerReceived = true;
}

// receive bms hardware version info
// cmd id = 194
void McuManager::procBMSHwVer(const tinyros::atris_msgs::CanPkg &msg)
{
    bms_hw_version_ = msg.data_s;
    log_info("%s recv bms hardware version : %s",__FUNCTION__, bms_hw_version_.c_str());
    isBMSHwVerReceived = true;
}

// receive bms software version info
// cmd id = 196
void McuManager::procBMSSwVer(const tinyros::atris_msgs::CanPkg &msg)
{
    bms_sw_version_ = msg.data_s;
    log_info("%s recv bms software version : %s",__FUNCTION__, bms_sw_version_.c_str());
    isBMSSwVerReceived = true;
}

void McuManager::procChassisSetTransportModeResp(const tinyros::atris_msgs::CanPkg &msg)
{
    log_info("%s get chassis set in transport mode response", __FUNCTION__);
    if(msg.data_i[0] == 1)
    {
        log_warn("%s set robot in air transport mode", __FUNCTION__);
    }
    else
    {
        log_warn("%s unset robot in air transport mode", __FUNCTION__);
    }
}

// receive chassis break reason info from chassis controller
// cmd id = 164
// cycle = 200ms

void McuManager::procChassisStopReasonInfo(const tinyros::atris_msgs::CanPkg &msg)
{
    int brake_enable_status;
    int brake_reason_detail;
    static long chassis_brake_reason_recv_last = 0;
    struct timeval chassis_brake_reason_recv_this;
    gettimeofday(&chassis_brake_reason_recv_this,NULL);

    // 0x00 means the button is not pressed
    // 0x01 means the button is pressed
    brake_enable_status = msg.data_i[0];
    //log_info("emergency button status : %d", emergency_button_status);

    // 0x00 means the anti collision bar is not triggered 
    // 0x01 means the anti collision bar is triggered 
    // 0x02 means the anti collision bar is vacant

    brake_reason_detail = msg.data_i[1];
    //log_info("collision bar status : %d", collision_bumper_status);
    // publish the bumper status message every 5 secs

    if((brake_enable_status_ != brake_enable_status) || (chassis_brake_reason_recv_this.tv_sec - chassis_brake_reason_recv_last) > 5)
    {
        //log_warn("%s recv change of brake trigger info changed or interval > 5s : brake enable status = %d, brake reason detail = %d", __FUNCTION__, brake_enable_status, brake_reason_detail);
        //log_warn("%s brake triggered reason = %d , software : %d , button : %d , bumper : %d",__FUNCTION__, brake_reason_detail, brake_reason_detail & 0x01, (brake_reason_detail >> 1) & 0x01, (brake_reason_detail >> 2) & 0x01);
        //log_warn("%s time interval = %ld secs",__FUNCTION__, chassis_brake_reason_recv_this.tv_sec - chassis_brake_reason_recv_last);
        Json::Value root;
        atris_msgs::RobotInfo rbtInfo_brake_reason;
        Json::FastWriter fw;
        root["robot_info"]["brake"]["source"]["can"] = brake_reason_detail & 0x01;
        root["robot_info"]["brake"]["source"]["button"] = (brake_reason_detail >> 1) & 0x01;
        root["robot_info"]["brake"]["source"]["front_bumper"] = (brake_reason_detail >> 2) & 0x01;

        rbtInfo_brake_reason.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo_brake_reason);
        chassis_brake_reason_recv_last = chassis_brake_reason_recv_this.tv_sec;
        brake_enable_status_ = brake_enable_status;
    }
}

// receive machine status, 1 is standby and 0 is work mode
// cmd id = 124
// cycle = 1s

void McuManager::procChassisMachineStatus(const tinyros::atris_msgs::CanPkg &msg)
{
    //log_info("%s",__FUNCTION__);
    Json::Value root;
    atris_msgs::RobotInfo rbtInfo_machine_status;
    Json::FastWriter fw;
    root["robot_info"]["base"]["machine_status"] = msg.data_i[0];

    rbtInfo_machine_status.json = fw.write(root);
    diag_info_pub_.publish(rbtInfo_machine_status);
}

// receive temperature and humidity info from chassis controller
// cmd id = 146
// cycle = 300ms

void McuManager::procTempHumiInfo(const tinyros::atris_msgs::CanPkg &msg)
{
    //log_info("%s recv cmd id : %d", __FUNCTION__, msg.cmd);
    static long chassis_temp_humi_recv_last = 0;
    struct timeval chassis_temp_humi_recv_this;
    gettimeofday(&chassis_temp_humi_recv_this,NULL);
    
    #if 0
    log_info("temperature = %f",msg.data_i[0] * 0.1);
    log_info("humidity = %f",msg.data_i[0] * 0.1);
    log_info("NTC1 = %d",msg.data_i[2]);
    log_info("NTC2 = %d",msg.data_i[3]);
    log_info("NTC3 = %d",msg.data_i[4]);
    log_info("NTC4 = %d",msg.data_i[5]);
    #endif

    if((chassis_temp_humi_recv_this.tv_sec - chassis_temp_humi_recv_last) > 5)
    {
        Json::Value root;
        atris_msgs::RobotInfo rbtInfo_temp_humi;
        Json::FastWriter fw;
        root["robot_info"]["temp_humi"]["chassis_temp"] = msg.data_i[0] * 0.1;
        root["robot_info"]["temp_humi"]["chassis_humi"] = msg.data_i[1] * 0.1;
        root["robot_info"]["temp_humi"]["chassis_NTC1"] = msg.data_i[2];
        root["robot_info"]["temp_humi"]["chassis_NTC2"] = msg.data_i[3];
        root["robot_info"]["temp_humi"]["chassis_NTC3"] = msg.data_i[4];
        root["robot_info"]["temp_humi"]["chassis_NTC4"] = msg.data_i[5];

        rbtInfo_temp_humi.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo_temp_humi);
        chassis_temp_humi_recv_last = chassis_temp_humi_recv_this.tv_sec;
    }
}

// process battery info reported from chassis controller
// cmd = 100

void McuManager::procBatMonInfo(const tinyros::atris_msgs::CanPkg &msg)
{
    // publish the battery diag info out to diag module
    // we should recv this message every 1 sec, it is slow enough
    Json::Value root;
    atris_msgs::RobotInfo rbtInfo_battery;
    Json::FastWriter fw;
    #if 0
    log_info("total battery voltage :%f", msg.data_i[0] * 0.1);
    log_info("total battery current : %f", (msg.data_i[1] - 5000) * 0.1);
    log_info("soc :%d", msg.data_i[2]);
    log_info("soh :%f", msg.data_i[3] * 0.1);
    log_info("single cell temp max :%d", msg.data_i[12] - 40);
    log_info("single cell temp min :%d", msg.data_i[13] - 40);
    log_info("charge indicator :%d", msg.data_i[24]);
    #endif


    root["robot_info"]["battery"]["voltage"] = msg.data_i[0] * 0.1;
    root["robot_info"]["battery"]["current"] = (msg.data_i[1] - 5000) * 0.1;
    root["robot_info"]["battery"]["level"] = msg.data_i[2];
    root["robot_info"]["battery"]["health"] = (int)(msg.data_i[3] * 0.1);                
    root["robot_info"]["battery"]["nominal_voltage"] = msg.data_i[4] * 0.1;
    root["robot_info"]["battery"]["nominal_current"] = msg.data_i[5] * 0.1;            
    root["robot_info"]["battery"]["voltage_max"] = msg.data_i[6];
    root["robot_info"]["battery"]["voltage_min"] = msg.data_i[7];
    root["robot_info"]["battery"]["voltage_max_num"] = msg.data_i[8];
    root["robot_info"]["battery"]["voltage_min_num"] = msg.data_i[9];
    root["robot_info"]["battery"]["voltage_max_serial_num"] = msg.data_i[10];
    root["robot_info"]["battery"]["voltage_min_serial_num"] = msg.data_i[11];
    root["robot_info"]["battery"]["temp_max"] = msg.data_i[12] - 40;
    root["robot_info"]["battery"]["temp_min"] = msg.data_i[13] - 40;
    root["robot_info"]["battery"]["temp_max_pack_num"] = msg.data_i[14];
    root["robot_info"]["battery"]["temp_min_pack_num"] = msg.data_i[15];
    root["robot_info"]["battery"]["temp_max_pos"] = msg.data_i[16];
    root["robot_info"]["battery"]["temp_min_pos"] = msg.data_i[17];
    root["robot_info"]["battery"]["life_percent"] = msg.data_i[18];
    root["robot_info"]["battery"]["charge_mos_status"] = msg.data_i[19];
    root["robot_info"]["battery"]["discharge_mos_status"] = msg.data_i[20];
    root["robot_info"]["battery"]["external_io_status"] = msg.data_i[21];
    root["robot_info"]["battery"]["charge_discharge_cycles"] = msg.data_i[22];
    root["robot_info"]["battery"]["shutdown_indicator"] = msg.data_i[23];
    root["robot_info"]["battery"]["status"] = msg.data_i[24]; // charging status
    root["robot_info"]["battery"]["warning_status"] = msg.data_i[25];
    root["robot_info"]["battery"]["alarm"] = msg.data_i[26]; // alarm level
    root["robot_info"]["battery"]["warning_low"] = msg.data_i[27];
    root["robot_info"]["battery"]["warning_mid"] = msg.data_i[28];
    root["robot_info"]["battery"]["warning_high"] = msg.data_i[29];
    
    rbtInfo_battery.json = fw.write(root);
    diag_info_pub_.publish(rbtInfo_battery);
}

int McuManager::sendChassisUnlockNonBlock(int status)
{
    log_info("%s status = %d",__FUNCTION__, status);
    tinyros::atris_msgs::CanPkg request;
    
    request.cmd = ROBOT_TO_CHASSIS_EMERGE_STOP_REQ;
    request.data_i[0] = 0x01; // only use set command
    request.data_i[1] = status;

    chassis_controller_service_pub_.publish(&request);

    return 0;
}

// request to chassis controller to have a software emergency stop
// chassis will not response speed that sent to it
// cmd id = 161
// resp cmd id = 162
// synchronos
int McuManager::setChassisSoftEmergeStop(int status)
{
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);
    bool bRet;
    bRet = removeChassisDupRequest(ROBOT_TO_CHASSIS_EMERGE_STOP_REQ);
    if(bRet)
    {
        log_warn("%s remove duplicate request %d from response map success...", __FUNCTION__, ROBOT_TO_CHASSIS_EMERGE_STOP_REQ);
    }
    else
    {
        log_warn("%s no duplicate request from response map",__FUNCTION__);
    }

    request.cmd = ROBOT_TO_CHASSIS_EMERGE_STOP_REQ;
    request.data_i[0] = 0x01; // only use set command
    request.data_i[1] = status;

    CHASSIS_CONTROLLER_CMD_SEND(5);

    // check the request ack see if it is success

    if(response.data_i[0] != status)
    {
        log_error("%s set chassis soft emergency stop failed , request status = %d , response = %d",__FUNCTION__, status , response.data_i[0]);
        return CTRL_REQ_INNER_ERROR;
    }
    else
    {
        log_info("%s set chassis soft emergency stop success, request status = %d , response = %d", __FUNCTION__, status , response.data_i[0]);
    }

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);
    return CTRL_REQ_SUCCESS;
}

// query battery log from chassis controller
// cmd id = 101
// resp cmd id = 102
int McuManager::queryBatteryLog(char * pcBuf)
{
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);
    bool bRet;
    bRet = removeChassisDupRequest(QUERY_BATTERY_LOG_CMD_REQ);
    if(bRet)
    {
        log_warn("%s remove duplicate request %d from response map success...", __FUNCTION__, QUERY_BATTERY_LOG_CMD_REQ);
    }
    else
    {
        log_warn("%s no duplicate request from response map",__FUNCTION__);
    }

    request.cmd = QUERY_BATTERY_LOG_CMD_REQ;
    request.data_i[0] = 0; // query command

    // a little bit longer waited for battery log from chassis monitor
    CHASSIS_CONTROLLER_CMD_SEND(10);

    // TODO::
    // deal with message response from chassis controller
    // save the log to file

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

    return CTRL_REQ_SUCCESS;
}

// just response to chassis the same message operation back
// tell mcu that we received the message
// mcu -> robot cmd id = 112
// robot -> mcu resp cmd id = 111 
int McuManager::chassisNotifyRobotOperation(const tinyros::atris_msgs::CanPkg &msg)
{
    tinyros::atris_msgs::CanPkg request;
    //log_error("%s data_i[0] = %d", __FUNCTION__, msg.data_i[0]);

    request.cmd = CHASSIS_CONTROLLER_NOTIFY_ROBOT_OP_RESP;
    // ack the same command result back to mcu
    request.data_i[0] = msg.data_i[0];
    chassis_controller_service_pub_.publish(&request);
    // TODO::
    // if we need to do some other operation after we receive the message
    // do it here
    if(msg.data_i[0] == 0x01)
    {
        log_info("%s recv atris power off cmd after power off button is pressed , cmd id = %d", __FUNCTION__, msg.data_i[0]);
        procNormalPowerOff();
    }
    else if(msg.data_i[0] == 0x02)
    {
        log_info("%s recv atris recharge into standby mode request , cmd id = %d", __FUNCTION__, msg.data_i[0]);
        procChassisRechargeStandby();
        
    }
    else if(msg.data_i[0] == 0x03)
    {
        log_info("%s recv mcu wake up robot command , cmd id = %d", __FUNCTION__, msg.data_i[0]);
    }
    else if(msg.data_i[0] == 0x04)
    {
        log_info("%s recv mcu notify charger plugged into standby mode request, cmd id = %d",__FUNCTION__, msg.data_i[0]);
        procChassisChargerPlugged();
    }
    else
    {
        log_info("%s unknown message received from mcu msg cmd id = %d \r\n",__FUNCTION__, msg.data_i[0]);
    }

    return 0;
}

// do robot operation request to chassis controller(shutdown standby)
// operation :
// 0x01 means normal shut down
// 0x02 means recharging shut down
// 0x03 standby mode
// 0x04 wait up from standby mode 
// robot -> mcu cmd id = 113
// mcu -> robot resp cmd id = 114

int McuManager::doRobotOpReq(int op)
{
    log_info("%s op = %d",__FUNCTION__, op);
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;
    bool bRet;

    request.cmd = ROBOT_INFORM_CHASSIS_CONTROLLER_OP_REQ;
    request.data_i[0] = op;
    log_info("%s cmd id = %d", __FUNCTION__, request.cmd);

    chassis_controller_service_pub_.publish(&request);

    return CTRL_REQ_SUCCESS;
}

int McuManager::notifyRobotInPosition(int charge_result)
{
    log_info("%s stop moving result = %d",__FUNCTION__, charge_result);
    tinyros::atris_msgs::CanPkg request;
    //tinyros::atris_msgs::CanPkg response;
    bool bRet;
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);

    request.cmd = ROBOT_TO_CHASSIS_RECHARGE_IN_POSITION_REQ;
    request.data_i[0] = charge_result;

    chassis_controller_service_pub_.publish(&request);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

    return CTRL_REQ_SUCCESS;
}

// robot inform mcu to do the recharging request(on pile or leave pile)
// input param 
// op : 
// 0x01 means leave pile
// robot -> mcu cmd id = 115
// mcu -> robot resp cmd id = 116

int McuManager::startRobotRechargeOperation(int op)
{
    log_info("%s op = %d",__FUNCTION__, op);
    tinyros::atris_msgs::CanPkg request;
    //tinyros::atris_msgs::CanPkg response;
    bool bRet;
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);

    request.cmd = ROBOT_TO_CHASSIS_RECHARGE_REQ;
    request.data_i[0] = op;

    chassis_controller_service_pub_.publish(&request);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

    return CTRL_REQ_SUCCESS;
}

// chassis notify charging result back to robot
// mcu -> robot cmd id = 118
// robot -> mcu resp cmd id = 117

int McuManager::chassisNotifyRechargeResult(const tinyros::atris_msgs::CanPkg &msg)
{
    log_info("%s received message cmd id : %d", __FUNCTION__, msg.cmd);
    //printChassisControllerReporMsg(msg);
    int recharge_status_resp;
    int recharge_result_resp;
    int recharge_failed_reason;
    // recharge status : electrode detect or leave pile process finish indication
    recharge_status_resp = msg.data_i[0];
    log_info("%s recharge status : %d\r\n", __FUNCTION__, recharge_status_resp);
    // recharge result : 0x00 success 0x02 failed
    recharge_result_resp = msg.data_i[1];
    log_info("%s recharge result : %d\r\n", __FUNCTION__, recharge_result_resp);

    // TODO :: recharge failed reason
    recharge_failed_reason = msg.data_i[2];
    log_info("%s recharge failed reason : %d\r\n", __FUNCTION__, recharge_failed_reason);

    // navigation need the result
    // we publish it to navigation module
    atris_msgs::PowerChargeCmd charge_status_notify;

    charge_status_notify.charge_msg_type = atris_msgs::PowerChargeCmd::CHASSIS_NOTIFY_CHARGE_STATUS;
    charge_status_notify.charge_status = recharge_status_resp;
    charge_status_notify.charge_result = recharge_result_resp;
    charge_status_notify.charge_failed_reason = recharge_failed_reason;

    charge_response_pub_.publish(charge_status_notify);

    tinyros::atris_msgs::CanPkg request;
    request.cmd = CHASSIS_TO_ROBOT_RECHARGE_RESULT_RESP;
    request.data_i[0] = msg.data_i[0] - 1;
    // just send command and no data ack to mcu
    //if(recharge_status_resp != 0x01)
    //{
        // if we receive electrode contact notification , do not send the response back to mcu
        chassis_controller_service_pub_.publish(&request);
    //}

    return CTRL_REQ_SUCCESS;
}

// query chassis controlled voltage source status
// output param control_source_1 , control_source_2
// 1 means open 0 means closed
// robot -> mcu cmd id = 131
// mcu -> robot resp cmd id = 132

int McuManager::queryChassisControlSourceState(int & control_source_1, int & control_source_2)
{
    std::unique_lock<std::mutex> lock(chassis_controlled_source_mutex_);
    bool bRet;

    bRet = removeChassisDupRequest(CHASSIS_CONTROLLED_SOURCE_REQ);
    if(bRet)
    {
        log_warn("%s found duplicate query set controlled voltage source 1 cmd, remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s chassis controlled source query set cmd not in map", __FUNCTION__);
    }

    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = CHASSIS_CONTROLLED_SOURCE_REQ;
    request.data_i[0] = 0x01; // query command

    CHASSIS_CONTROLLER_CMD_SEND(5);

    log_info("%s query chassis controlled source info success... control source 1 : %d , control source 2 : %d", __FUNCTION__, response.data_i[1] & 0x01, response.data_i[1] & 0x02);

    control_source_1 = response.data_i[1] & 0x01;
    control_source_2 = response.data_i[1] & 0x02;

    return CTRL_REQ_SUCCESS;
}

// for internal use to query the chassis control voltage source status

int McuManager::queryChassisControlSource(int & control_source_1, int & control_source_2)
{
    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = CHASSIS_CONTROLLED_SOURCE_REQ;
    request.data_i[0] = 0x01; // query command

    CHASSIS_CONTROLLER_CMD_SEND(5);

    log_info("%s query chassis controlled source info success... control source 1 : %d , control source 2 : %d", __FUNCTION__, response.data_i[1] & 0x01, response.data_i[1] & 0x02);

    control_source_1 = response.data_i[1] & 0x01;
    control_source_2 = response.data_i[1] & 0x02;

    return CTRL_REQ_SUCCESS;
}

// set chassis controlled voltage source1 status
// intput param status
// 1 means open 0 means closed
// robot -> mcu cmd id = 131
// mcu -> robot resp cmd id = 132

int McuManager::setChassisControlSource1(int status)
{
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);
    // multithread protection of the controlled voltage source internal state
    std::unique_lock<std::mutex> lock(chassis_controlled_source_mutex_);
    // first judge control command val is in range
    if(status < 0 || status > 1)
    {
        log_error("%s chassis set control source 1 input parameter invalid!!!", __FUNCTION__);
        return CTRL_REQ_PARAM_INVALID;
    }

    int control_source_one, control_source_two;
    int iRet;
    bool bRet;

    bRet = removeChassisDupRequest(CHASSIS_CONTROLLED_SOURCE_REQ);
    if(bRet)
    {
        log_warn("%s found duplicate query set controlled voltage source 1 cmd, remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s chassis controlled source query set cmd not in map", __FUNCTION__);
    }

    // query the controlled source 1 status info first
    iRet = queryChassisControlSource(control_source_one, control_source_two);
    if(0 > iRet)
    {
        log_error("%s query chassis controlled source 1 status failed!!!",__FUNCTION__);
        return iRet;
    }

    if(control_source_one == status)
    {
    	log_warn("%s chassis controller control source 1 already at state : %d", __FUNCTION__, control_source_one);
    	return CTRL_REQ_SUCCESS;
    }

    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = CHASSIS_CONTROLLED_SOURCE_REQ;
    request.data_i[0] = 1; // set command
    request.data_i[1] = status + (control_source_two << 1);

    CHASSIS_CONTROLLER_CMD_SEND(5);

    chassis_controlled_source1_ = response.data_i[0] & 0x01;
    chassis_controlled_source2_ = response.data_i[0] & 0x02;

    if(status != (response.data_i[0] & 0x01))
    {
        log_error("%s set controlled voltage source 1 to status : %d failed!!!",__FUNCTION__, status);
        return CTRL_REQ_INNER_ERROR;
    }

    log_info("%s set controlled voltage source 1 to status : %d success...",__FUNCTION__, response.data_i[0] & 0x01);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);
	return CTRL_REQ_SUCCESS;
}

// set chassis controlled voltage source2 status
// intput param status
// 1 means open 0 means closed
// robot -> mcu cmd id = 131
// mcu -> robot resp cmd id = 132

int McuManager::setChassisControlSource2(int status)
{
    struct timeval start,end;
    float duration;
    gettimeofday(&start,NULL);

    // multithread protection of the controlled voltage source internal state
    std::unique_lock<std::mutex> lock(chassis_controlled_source_mutex_);
    // first judge control command val is in range
    if(status < 0 || status > 1)
    {
        log_error("%s chassis set control source 2 status input parameter invalid!!!", __FUNCTION__);
        return CTRL_REQ_PARAM_INVALID;
    }

    int control_source_one, control_source_two;
    int iRet;
    bool bRet;

    bRet = removeChassisDupRequest(CHASSIS_CONTROLLED_SOURCE_REQ);
    if(bRet)
    {
        log_warn("%s found duplicate query set controlled voltage source 2 cmd , remove ok",__FUNCTION__);
    }
    else
    {
        log_info("%s chassis controlled source query set cmd not in map", __FUNCTION__);
    }

    // query the controlled source 2 status info first
    iRet = queryChassisControlSource(control_source_one, control_source_two);
    if(0 > iRet)
    {
        log_error("%s query chassis controlled source 2 status failed!!!",__FUNCTION__);
        return iRet;
    }

    if(control_source_two == status)
    {
    	log_warn("%s chassis controller control source 2 already at state : %d", __FUNCTION__, control_source_two);
    	return CTRL_REQ_SUCCESS;
    }

    tinyros::atris_msgs::CanPkg request;
    tinyros::atris_msgs::CanPkg response;

    request.cmd = CHASSIS_CONTROLLED_SOURCE_REQ;
    request.data_i[0] = 1; // set command
    request.data_i[1] = (status << 1) + control_source_one;

    CHASSIS_CONTROLLER_CMD_SEND(5);

    chassis_controlled_source1_ = response.data_i[0] & 0x01;
    chassis_controlled_source2_ = response.data_i[0] & 0x02;

    if(status != (response.data_i[0] & 0x02))
    {
        log_error("%s set chassis controlled voltage source 2 to status : %d failed!!!",__FUNCTION__, status);
        return CTRL_REQ_INNER_ERROR;
    }

    log_info("%s set chassis controlled voltage source 2 to status : %d success...",__FUNCTION__, response.data_i[0] & 0x02);

    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);
    return CTRL_REQ_SUCCESS;
}


int McuManager::setChassisTransportMode(int status)
{
    if(status < 0 || status > 1)
    {
        log_error("%s parameter invalid",__FUNCTION__);
        return CTRL_REQ_PARAM_INVALID;
    }

    log_info("%s status = %d",__FUNCTION__, status);
    tinyros::atris_msgs::CanPkg request;
    
    request.cmd = ROBOT_TO_CHASSIS_SET_TRANSPORT_MODE_REQ;
    request.data_i[0] = status;

    chassis_controller_service_pub_.publish(&request);
    return CTRL_REQ_SUCCESS;
}

// get system current millisecond time
// return long long
long long McuManager::getsysmstime(void)
{
    timeval tv;
    gettimeofday(&tv, NULL);
    return ((long long)tv.tv_sec) * 1000 + tv.tv_usec / 1000;
}

bool McuManager::on_recv_peripheral_device_ctrl(atris_msgs::PeripheralControl::Request& req,
    atris_msgs::PeripheralControl::Response& res) {
    int iRet;
    if (req.cmd == atris_msgs::PeripheralControl::Request::LIGHT_COLOR_CONTROL) {
        iRet = setIndicatorLightColor(req.color);
        if(iRet < 0)
        {
            res.result = false;
        }
        else
        {
            res.result = true;
        }
    } else if (req.cmd == atris_msgs::PeripheralControl::Request::LIGHT_STYLE_CONTROL) {
        iRet = setIndicatorLightStyle(req.style);
        if(iRet < 0)
        {
            res.result = false;
        }
        else
        {
            res.result = true;
        }
    } else if (req.cmd == atris_msgs::PeripheralControl::Request::LIGHT_BRIGHTNESS_CONTROL) {
        iRet = setIndicatorLightBrightness(req.brightness);
        if(iRet < 0)
        {
            res.result = false;
        }
        else
        {
            res.result = true;
        }

    } else if (req.cmd == atris_msgs::PeripheralControl::Request::LIGHT_CONTROL_RELEASE) {
        iRet = releaseIndicatorLightControl();
        if(iRet < 0)
        {
            res.result = false;
        }
        else
        {
            res.result = true;
        }
    }

    return true;
}

// on receive peripheral device control

void McuManager::on_recv_peripheral_ctrl(const atris_msgs::SignalMessage &msg)
{
    log_info("[%s]:%s", __FUNCTION__, msg.title.c_str());
    Json::Reader reader;
    Json::Value req, resp;
    std::string resp_name;
    int iRet;

    resp["id"] = msg.msgID;
    resp["timestamp"] = msg.timestamp;
    resp["result"] = "success";

    if (msg.title == "request_set_indicator_style") 
    {
        reader.parse(msg.msg, req);
        int light_style;
        resp_name = "response_set_indicator_style";
        if (req["content"]["style"].isNull()) 
        {
            log_error("%s request set indicator light style , style field is NULL",__FUNCTION__);
            resp["result"] = "fail_invalid_data";
        }
        else
        {
            /*
            TODO 
            */
            light_style = req["content"]["style"].asInt();
            log_info("%s set indicator light style , style type = %s",__FUNCTION__, get_light_style_str(light_style).c_str());
            iRet = setIndicatorLightStyle(req["content"]["style"].asInt());
            if(iRet < 0)
            {
                // for debug
                log_error("%s control set indicator light style failed !!! iRet : %d", __FUNCTION__, iRet);
                resp["result"] = "fail_internal_error";
            }
            else
            {
                log_info("%s control set indicator light style success.... iRet : %d", __FUNCTION__, iRet);
                resp["result"] = "success";
            }
        }

        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }
    else if(msg.title == "request_set_indicator_color")
    {
        reader.parse(msg.msg, req);
        resp_name = "response_set_indicator_color";
        int light_color;
        if (req["content"]["color"].isNull()) 
        {
            log_error("%s request set indicator color , color field is NULL",__FUNCTION__);
            resp["result"] = "fail_invalid_data";
        } 
        else
        {
            /*
            TODO 
            */
            light_color = req["content"]["color"].asInt();
            log_info("%s set indicator light color , color id = %s",__FUNCTION__, get_light_color_str(light_color).c_str());
            iRet = setIndicatorLightColor(req["content"]["color"].asInt());
            if(iRet < 0)
            {
                // for debug
                log_error("%s control set indicator light color failed !!! iRet : %d", __FUNCTION__, iRet);
                resp["result"] = "fail_internal_error";
            }
            else
            {
                log_info("%s control set indicator light color success.... iRet : %d", __FUNCTION__, iRet);
                resp["result"] = "success";
            }
        }

        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }
    else if(msg.title == "request_set_indicator_brightness")
    {
        reader.parse(msg.msg, req);
        resp_name = "response_set_indicator_brightness";
        int light_brightness;
        if (req["content"]["brightness"].isNull()) 
        {
            log_error("%s request set indicator brightness , brightness field is NULL",__FUNCTION__);
            resp["result"] = "fail_invalid_data";
        } 
        else
        {
            /*
            TODO 
            */
            light_brightness = req["content"]["brightness"].asInt();
            log_info("%s set indicator light brightness , brightness = %d",__FUNCTION__, light_brightness);
            iRet = setIndicatorLightBrightness(req["content"]["brightness"].asInt());
            if(iRet < 0)
            {
                // for debug
                log_error("%s control set indicator light brightness failed !!! iRet : %d", __FUNCTION__, iRet);
                resp["result"] = "fail_internal_error";
            }
            else
            {
                log_info("%s control set indicator light brightness success.... iRet : %d", __FUNCTION__, iRet);
                resp["result"] = "success";
            }
        }

        Utils::get_instance()->responseResult(msg, resp, resp_name);

    }
    else if(msg.title == "request_release_indicator_light_control")
    {
        reader.parse(msg.msg, req);
        resp_name = "response_release_indicator_light_control";
        if(!req["content"].isNull() && !req["content"]["id"].isNull())
        {
            log_warn("%s request release indicator light control",__FUNCTION__);
	        iRet = releaseIndicatorLightControl();
	        if(iRet < 0)
	        {
                resp["result"] = "fail_internal_error";
	        }
	        else
	        {
                resp["result"] = "success";
	        }
        }
        else
        {
            log_error("%s fail invalid data", __FUNCTION__);
            resp["result"] = "fail_invalid_data";
        }

        Utils::get_instance()->responseResult(msg, resp, resp_name);
    }
    else if(msg.title == "request_send_robot_in_position") // for testing use
    {
        // just for testing purpose
        // we fake to send the result to mcu for test
        int in_position_result;
        reader.parse(msg.msg, req);
        resp_name = "response_send_robot_in_position";
        int result;
        if(req["content"]["action"].isNull())
        {
            log_error("action field can not be empty");
            resp["result"] = "fail_invalid_data";
        }
        else
        {
            in_position_result = req["content"]["action"].asInt();
            log_info("%s in position result : %d", __FUNCTION__, in_position_result);
            notifyRobotInPosition(in_position_result);
            resp["result"] = "success";
            // just test , do not need to ack the result
        }

        // send no response to web
    }
    else if(msg.title == "request_send_leave_pile") // for testing use
    {
        reader.parse(msg.msg, req);
        resp_name = "response_send_leave_pile";
        // no parameter
        startRobotRechargeOperation(0x01);
        // result is always success...
        resp["result"] = "success";
        // jsut test , do not need to ack the result
    }
    else if(msg.title == "request_emergency_stop")
    {
        reader.parse(msg.msg, req);
        if(!req["content"].isNull() && !req["content"]["id"].isNull()
                && !req["content"]["switch"].isNull())
        {
            int state = req["content"]["switch"].asInt();
            resp["switch"] = Json::Value(state);
            log_info("%s title = %s state = %d",__FUNCTION__, msg.title.c_str(), state);

            if(state == 0)
            {
                log_error("%s request software emergency stop released!!!",__FUNCTION__);
                iRet = setChassisSoftEmergeStop(state);
                if(iRet < 0)
                {
                    log_error("%s request soft emergency stop released failed , iRet = %d", __FUNCTION__, iRet);
                    resp["result"] = "failed_internal_error";
                }
                else
                {
                    resp["result"] = "success";
                }
            }
            else
            {
                log_error("%s request software emergency stop pushed!!!",__FUNCTION__);
                iRet = setChassisSoftEmergeStop(state);
                if(iRet < 0)
                {
                    log_error("%s request soft emergency stop pushed failed , iRet = %d", __FUNCTION__, iRet);
                    resp["result"] = "failed_internal_error";
                }
                else
                {
                    resp["result"] = "success";
                }

            }
            Utils::get_instance()->responseResult(msg, resp, "response_emergency_stop");
        }
        else
        {
            log_error("%s fail invalid data", __FUNCTION__);
            resp["result"] = "fail_invalid_data";
            Utils::get_instance()->responseResult(msg, resp, "response_emergency_stop");
        }
    
    }
    else if(msg.title == "request_robot_standby_mode") // remote standby mode
    {
        reader.parse(msg.msg, req);
        if(!req["content"].isNull() && !req["content"]["id"].isNull())
        {
            //resp["result"] = "success";
            log_warn("%s request set mcu in standby mode",__FUNCTION__);
            //doRobotOpReq(0x03); // request mcu in standby mode
            iRet = reqRobotInStandbyMode();
            if(iRet < 0)
            {
                resp["result"] = "fail_internal_error";
            }
            else
            {
                resp["result"] = "success";
            }

            Utils::get_instance()->responseResult(msg, resp, "response_robot_standby_mode");
        }
        else
        {
            log_error("%s request set mcu in standby mode , fail invalid data", __FUNCTION__);
            resp["result"] = "fail_invalid_data";
            Utils::get_instance()->responseResult(msg, resp, "response_robot_standby_mode");
        }
    }
    else if(msg.title == "request_robot_wake_up") // remote wake up from standby mode
    {
        reader.parse(msg.msg, req);
        if(!req["content"].isNull() && !req["content"]["id"].isNull())
        {
            //resp["result"] = "success";
            log_warn("%s request wake up mcu from standby mode",__FUNCTION__);
            //doRobotOpReq(0x04); // request mcu to wake up from standby mode
            iRet = reqRobotStandbyWakeup();
            if(iRet < 0)
            {
                resp["result"] = "fail_internal_error";
            }
            else
            {
                resp["result"] = "success";
            }

            Utils::get_instance()->responseResult(msg, resp, "response_robot_wake_up");
        }
        else
        {
            log_error("%s request wake up mcu from standby mode , fail invalid data", __FUNCTION__);
            resp["result"] = "fail_invalid_data";
            Utils::get_instance()->responseResult(msg, resp, "response_robot_wake_up");
        }
    }
    else if(msg.title == "request_robot_transport_mode")
    {
        reader.parse(msg.msg, req);
        if(!req["content"].isNull() && !req["content"]["id"].isNull()
                && !req["content"]["switch"].isNull())
        {
            int state = req["content"]["switch"].asInt();
            log_info("%s title = %s transport mode = %d",__FUNCTION__, msg.title.c_str(), state);

            if(state == 0)
            {
                log_error("%s request unset robot transport mode!!!",__FUNCTION__);
                iRet = setChassisTransportMode(state);
                if(iRet < 0)
                {
                    log_error("%s request unset robot transport mode failed , iRet = %d", __FUNCTION__, iRet);
                    resp["result"] = "failed_internal_error";
                }
                else
                {
                    resp["result"] = "success";
                }
            }
            else
            {
                log_error("%s request set robot in transport mode",__FUNCTION__);
                iRet = setChassisTransportMode(state);
                if(iRet < 0)
                {
                    log_error("%s request set robot in transport mode failed , iRet = %d", __FUNCTION__, iRet);
                    resp["result"] = "failed_internal_error";
                }
                else
                {
                    resp["result"] = "success";
                }

            }

            Utils::get_instance()->responseResult(msg, resp, "response_robot_transport_mode");
        }
        else
        {
            log_error("%s fail invalid data", __FUNCTION__);
            resp["result"] = "fail_invalid_data";
            Utils::get_instance()->responseResult(msg, resp, "response_robot_transport_mode");
        }        
    }
}

int McuManager::reqRobotInStandbyMode(void)
{
    int ret;
    log_info("%s",__FUNCTION__);
    std::unique_lock<std::mutex> wlock(standby_mode_mutex_);

    doRobotOpReq(0x03);

    if(standby_mode_cv_.wait_until(wlock, std::chrono::system_clock::now() + std::chrono::milliseconds(5 * 1000)) == std::cv_status::timeout)
    {
        log_info("%s wait mcu response into standby mode cmd response failed!!!",__FUNCTION__);
        ret = -1;
    }
    else
    {

        log_info("%s wait mcu response into standby mode cmd response ok...",__FUNCTION__);
        ret = 0;
    }

    return ret;
}

int McuManager::reqRobotStandbyWakeup(void)
{
    int ret;
    log_info("%s",__FUNCTION__);
    std::unique_lock<std::mutex> wlock(wakeup_mutex_);

    doRobotOpReq(0x04);

    if(wakeup_cv_.wait_until(wlock, std::chrono::system_clock::now() + std::chrono::milliseconds(5 * 1000)) == std::cv_status::timeout)
    {
        log_info("%s wait mcu response standby mode wake up cmd response failed!!!",__FUNCTION__);
        ret = -1;
    }
    else
    {

        log_info("%s wait mcu response standby mode wake up cmd response ok...",__FUNCTION__);
        ret = 0;
    }

    return ret;
}

// 0 静止
// 1 呼吸
// 2 频闪
std::string McuManager::get_light_style_str(int style_id)
{
    std::string light_style_string = "";
    switch(style_id)
    {
        case 0x00:
            light_style_string = "static";
            log_info("%s set light style to static\r\n",__FUNCTION__);
        break;
        case 0x01:
            light_style_string = "breathe";
            log_info("%s set light style to breathe\r\n",__FUNCTION__);
        break;
        case 0x02:
            light_style_string = "flashing";
            log_info("%s set light style to flashing\r\n",__FUNCTION__);
        break;
        default:
            log_error("%s unknown light style\r\n",__FUNCTION__);
        break;

    }

    return light_style_string;
}
#if 0
RED = 1,
ORANGE = 2,
GREEN = 3,
CYAN = 4,
BLUE = 5,
PURPLE = 6,
WHITE = 7
#endif
std::string McuManager::get_light_color_str(int color_id)
{
    std::string light_color_string = "";
    switch(color_id)
    {
        case 0x01:
            light_color_string = "RED";
            log_info("%s set light color to red\r\n",__FUNCTION__);
        break;
        case 0x02:
            light_color_string = "ORANGE";
            log_info("%s set light color to orange\r\n",__FUNCTION__);
        break;
        case 0x03:
            light_color_string = "GREEN";
            log_info("%s set light color to green\r\n",__FUNCTION__);
        break;

        case 0x04:
            light_color_string = "CYAN";
            log_info("%s set light color to cyan\r\n",__FUNCTION__);
        break;
        case 0x05:
            light_color_string = "BLUE";
            log_info("%s set light color to blue\r\n",__FUNCTION__);
        break;
        case 0x06:
            light_color_string = "PURPLE";
            log_info("%s set light color to purple\r\n",__FUNCTION__);
        break;
        case 0x07:
            light_color_string = "WHITE";
            log_info("%s set light color to white\r\n",__FUNCTION__);
        break;

        default:
            log_error("%s unknown color\r\n",__FUNCTION__);
        break;

    }
    return light_color_string;
}

// on receive charge command request from navigation module
void McuManager::on_recv_navigation_charge_request(const atris_msgs::PowerChargeCmd &msg)
{
    switch(msg.charge_msg_type)
    {
        case atris_msgs::PowerChargeCmd::START_LEAVE_PILE:
            log_info("%s recv start leave pile command \r\n",__FUNCTION__);
            startRobotRechargeOperation(0x01);
        break;

        case atris_msgs::PowerChargeCmd::ROBOT_NOTIFY_IN_POSITION:
            log_info("%s robot notify it is stop moving and enable charging",__FUNCTION__);
            notifyRobotInPosition(msg.charge_status);
        
        default:
            log_error("%s unknown power charge command message type : %d\r\n",__FUNCTION__, msg.charge_msg_type);
        break;

    }
}

// get main control board upgrade status
std::string McuManager::getICPUpgradeStatus(void)
{
    char buffer[1024];
    FILE *fp = NULL;
    char *s = NULL;
    std::string status = SWUPGRADE_ICP_STANDBY;

    memset(buffer, '\0', sizeof(buffer));
    if((fp = popen("atris_printenv recovery_status", "r"))) {
        while(fgets(buffer, sizeof(buffer) - 1, fp)) {
            if((s = strstr(buffer, SWUPGRADE_ICP_SUCCESS))) {
                status = SWUPGRADE_ICP_SUCCESS;
                break;
            } else if ((s = strstr(buffer, SWUPGRADE_ICP_FAILED))) {
                status = SWUPGRADE_ICP_FAILED;
                break;
            } else if ((s = strstr(buffer, SWUPGRADE_ICP_STANDBY))) {
                status = SWUPGRADE_ICP_STANDBY;
                break;
            } else if ((s = strstr(buffer, SWUPGRADE_ICP_PROGRESS))) {
                status = SWUPGRADE_ICP_PROGRESS;
                break;
            }
        }

        log_info("%s status: %s", __FUNCTION__, status.c_str());
        pclose(fp);
    } else {
        log_error("%s popen failed", __FUNCTION__);
    }

    return status;
}

bool McuManager::doGetSwVersion(atris_msgs::GetSwVersion::Request& req,
  atris_msgs::GetSwVersion::Response& res) 
{
    res.imx_version = imx_version_;
    res.chassis_sw_ver = chassis_sw_version_;
    res.chassis_core_hw_ver = chassis_core_hw_version_;
    res.chassis_base_hw_ver = chassis_base_hw_version_;
    res.monitor_sw_ver = monitor_sw_version_;
    res.monitor_core_hw_ver = monitor_core_hw_version_;
    res.monitor_base_hw_ver = monitor_base_hw_version_;
    res.bms_hw_ver = bms_hw_version_;
    res.bms_sw_ver = bms_sw_version_;

    return true;
}

std::string McuManager::getImxVersion()
{
    FILE * fp;
    std::string result = "";
    char * line = NULL;
    size_t len = 0;

    if ((fp = fopen(SWUPGRADE_VERSION_FILE, "r")) != NULL) {
        getline(&line, &len, fp);
        if (line) {
            result = line;
            result.erase(result.find_last_not_of(" \t\f\v\n\r") + 1);
            result.erase(0, result.find_first_not_of(" \t\f\v\n\r"));
            free(line);
        }
        fclose(fp);
    }

    return result;
}

// create a thread to proc power off command in case blocked
void McuManager::procNormalPowerOff(void)
{
    boost::shared_ptr<boost::thread> poweroff_thread(new boost::thread(
                boost::bind(&McuManager::proc_poweroff_thread, this)));
}

// this is the only case we shutdown x86
void McuManager::proc_poweroff_thread(void)
{
    int iRet;
    if (!is_power_off_)
    {
        is_power_off_ = true;
    }
    else
    {
        log_warn("%s already in shutdown process just return",__FUNCTION__);
        return;
    }
    
    log_info("%s start do poweroff process.", __FUNCTION__);
    
    FILE *fp = NULL;
    //log_info("%s poweroff: %d", __FUNCTION__, status);
    log_warn("%s shutdown atris now ^_^\n\n", __FUNCTION__);
    if((fp = popen("shutdown -f", "r"))) 
    {
        pclose(fp);
        fp = NULL;
    }

    //double time = ros::Time::now().toSec();
    //double duration = 40;
    //while(1)
    //{
        #if 0
        if(!face_state){
            if(utils->check_network_state(Config::get_instance()->gs_ip.c_str())){
                log_info("%s gs device reachable.", __FUNCTION__);
                sleep(1);
            } else {
                log_info("%s gs device unreachable.", __FUNCTION__);
                face_state = true;
            }
        }

        if(!nav_state){
            if(utils->check_network_state(Config::get_instance()->ptz_box_ip.c_str())){
                log_info("%s face x86 device reachable.", __FUNCTION__);
            } else {
                log_info("%s face x86 device unreachable.", __FUNCTION__);
                nav_state = true;
            }
        }

        if(face_state && nav_state) {
            break;
        }
        
        double now = ros::Time::now().toSec();
        if (now < time) {
            time = now;
        }
        if (((now - time) > duration)) {
            log_warn("%s Waitting timeout.", __FUNCTION__);
            break;
        } else {
            sleep(1);
        }
        #endif
        // now we have nothing to close
        // just simply sleep 5 seconds
        sleep(30);
    //}
    
    iRet = send_poweroff_finish(); // in this function we synchronous wait mcu response
    if(iRet < 0)
    {
        log_error("%s send power off finish",__FUNCTION__);
    }
    else
    {
        log_info("%s send power off finish wait failed",__FUNCTION__);
    }
    


}

// recv mcu response robot shutdown or wake up cmd
// cmd id = 114 
void McuManager::chassisNotifyOperationResp(const tinyros::atris_msgs::CanPkg &msg)
{
    if(msg.data_i[0] == 0x01)
    {
        // normal shutdown resposne notification
        log_info("%s recv mcu poweroff resp, will shutdown in 10 seconds", __FUNCTION__);
        std::unique_lock<std::mutex> wlock(power_off_mutex_);
        power_off_cv_.notify_one();
    }
    else if(msg.data_i[0] == 0x02)
    {
        // recharge standby response notification
        log_info("%s recv mcu recharge and into standby mode resp", __FUNCTION__);
        std::unique_lock<std::mutex> wlock(recharge_standby_mutex_);
        recharge_standby_cv_.notify_one();
    }
    else if(msg.data_i[0] == 0x03)
    {
        // just print and do nothing
        log_info("%s recv mcu set standby mode resp", __FUNCTION__);
        std::unique_lock<std::mutex> wlock(standby_mode_mutex_);
        robot_mode_ = 1;
        standby_mode_cv_.notify_one();
    }
    else if(msg.data_i[0] == 0x04)
    {
        std::unique_lock<std::mutex> wlock(wakeup_mutex_);
        log_info("%s recv mcu wake up from standby mode resp", __FUNCTION__);
        robot_mode_ = 0;
        wakeup_cv_.notify_one();
    }
    else if(msg.data_i[0] == 0x05)
    {
        log_info("%s recv mcu charger plugged and into standby mode resp",__FUNCTION__);
        std::unique_lock<std::mutex> wlock(charger_standby_mutex_);
        charger_standby_cv_.notify_one();
    }
    else
    {
        log_error("%s unknown command response , id = %d", __FUNCTION__, msg.data_i[0]);
    }
}

// wait mcu send response
int McuManager::send_poweroff_finish(void)
{
    int ret;

    std::unique_lock<std::mutex> wlock(power_off_mutex_);
    doRobotOpReq(0x01);

    if(power_off_cv_.wait_until(wlock, std::chrono::system_clock::now() + std::chrono::milliseconds(5 * 1000)) == std::cv_status::timeout)
    {
        log_info("%s wait mcu response normal power off response failed!!!",__FUNCTION__);
        ret = -1;
    }
    else
    {

        log_info("%s wait mcu response normal power off response ok...",__FUNCTION__);
        ret = 0;
    }

    return ret;
}

// send recharge proc deal finish cmd to mcu
// wait recharge proc finish response from mcu
int McuManager::send_recharge_proc_finish(void)
{
    int ret;

    std::unique_lock<std::mutex> wlock(recharge_standby_mutex_);
    doRobotOpReq(0x02);

    if(recharge_standby_cv_.wait_until(wlock, std::chrono::system_clock::now() + std::chrono::milliseconds(5 * 1000)) == std::cv_status::timeout)
    {
        log_info("%s wait mcu response recharge into standby mode response failed!!!",__FUNCTION__);
        ret = -1;
    }
    else
    {

        log_info("%s wait mcu response recharge into standby mode response ok...",__FUNCTION__);
        ret = 0;
    }

    return ret;
}

// charger plugged and mcu into standyby mode
void McuManager::procChassisChargerPlugged(void)
{
    boost::shared_ptr<boost::thread> poweroff_thread(new boost::thread(
        boost::bind(&McuManager::proc_charger_plugged_thread, this)));
}

// recharge and mcu into standby mode
void McuManager::procChassisRechargeStandby(void)
{
    boost::shared_ptr<boost::thread> poweroff_thread(new boost::thread(
        boost::bind(&McuManager::proc_recharge_standby_thread, this)));
}

int McuManager::send_charger_plugged_proc_finish(void)
{
    int ret;

    std::unique_lock<std::mutex> wlock(charger_standby_mutex_);
    doRobotOpReq(0x05);

    if(charger_standby_cv_.wait_until(wlock, std::chrono::system_clock::now() + std::chrono::milliseconds(5 * 1000)) == std::cv_status::timeout)
    {
        log_info("%s wait mcu response charger plugged response into standby mode failed!!!",__FUNCTION__);
        ret = -1;
    }
    else
    {

        log_info("%s wait mcu response charger plugged into standby mode response ok...",__FUNCTION__);
        ret = 0;
    }

    return ret;
}

// just create a thread in case we need to do something for the charger plug standby mode
void McuManager::proc_charger_plugged_thread(void)
{
    int iRet;
    log_info("%s start do charger plugged standby process.", __FUNCTION__);

    if (!is_charger_standby_)
    {
        is_charger_standby_ = true;
    }
    else
    {
        log_warn("%s already in charger plugged process and just return",__FUNCTION__);
        return;
    }

    // do something unknown
    iRet = send_charger_plugged_proc_finish(); // in this function we synchronous wait mcu response
    if(iRet < 0)
    {
        log_error("%s send power off finish",__FUNCTION__);
    }
    else
    {
        log_info("%s send power off finish wait failed",__FUNCTION__);
    }

    is_charger_standby_ = false;
}

// process recharge and go into standby mode thread proc
void McuManager::proc_recharge_standby_thread(void)
{
    int iRet;
    log_info("%s start do robot recharge and chassis into standby process.", __FUNCTION__);

    if (!is_recharge_standby_)
    {
        is_recharge_standby_ = true;
    }
    else
    {
        log_warn("%s already in recharge process just return",__FUNCTION__);
        return;
    }

    // do something unknown
    // just leave blank here

    iRet = send_recharge_proc_finish(); // in this function we synchronous wait mcu response
    if(iRet < 0)
    {
        log_error("%s send recharge proc finish and get response failed!!!",__FUNCTION__);
    }
    else
    {
        log_info("%s send recharge proc finish and get response ok...",__FUNCTION__);
    }

    is_recharge_standby_ = false;
}

// receive upload log to hfs request from diagnostics module
void McuManager::on_recv_mcu_upload_log_request(const atris_msgs::McuLogCtrl& msg)
{
    if(msg.upload_log_msg == atris_msgs::McuLogCtrl::START_UPLOAD_LOG)
    {
        reqGetMcuLog(msg.board_name);
    }
    else
    {
        log_error("%s request get mcu log, request message type not correct, %d", __FUNCTION__, msg.upload_log_msg);
    }
}

void McuManager::reqGetMcuLog(const std::string & board_name)
{
    log_info("%s board name = %s",__FUNCTION__, board_name.c_str());
    tinyros::atris_msgs::CanPkg request;
    
    if(board_name == "chassis")
    {
        log_info("%s request get log from chassis controller",__FUNCTION__);
        request.cmd = CHASSIS_LOG_FILE_GET_REQ;
        chassis_controller_service_pub_.publish(&request);
    }
    else
    {
        log_info("%s request get log from system monitor",__FUNCTION__);
        request.cmd = MONITOR_LOG_FILE_GET_REQ;
        sys_monitor_service_pub_.publish(&request);
    }
}

void McuManager::procChassisUploadLogResp(const tinyros::atris_msgs::CanPkg &msg)
{
    log_info("%s receive chassis finish upload response",__FUNCTION__);
    atris_msgs::McuLogCtrl chassis_upload_log_resp;
    chassis_upload_log_resp.board_name = "chassis";
    chassis_upload_log_resp.upload_log_msg = atris_msgs::McuLogCtrl::FINISH_UPLOAD_LOG;

    mcu_upload_response_pub_.publish(chassis_upload_log_resp);
}

void McuManager::processMonitorUploadLogResp(const tinyros::atris_msgs::CanPkg &msg)
{
    log_info("%s receive system monitor finish upload response",__FUNCTION__);
    atris_msgs::McuLogCtrl monitor_upload_log_resp;
    monitor_upload_log_resp.board_name = "monitor";
    monitor_upload_log_resp.upload_log_msg = atris_msgs::McuLogCtrl::FINISH_UPLOAD_LOG;

    mcu_upload_response_pub_.publish(monitor_upload_log_resp);
}

bool McuManager::doGetRobotStandbyStatus(atris_msgs::GetStandbyMode::Request& req, 
    atris_msgs::GetStandbyMode::Response& res)
{
    res.standby_mode = robot_mode_;
}
