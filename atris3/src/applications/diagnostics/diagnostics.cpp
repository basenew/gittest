/*
 * diagnostics.cpp
 *
 *  Created on: 2018-6-25
 *      Author: fupj
 */
#include "log/log.h"
#include "diagnostics.h"
#include <sstream>
#include "eeprom/eeprom.h"
#include "can/can.h"
#include "config/config.h"
#include "tiny_ros/ros/time.h"
#include "tts_strings/tts_strings.h"
#include "transferfile/transferfile.h"
#include "task_manager/task_manager.h"
#include "utils/utils.h"
#include "imemory/atris_imemory_api.h"
#include "database/sqliteengine.h"
#include "list.h"
#include <sys/sysinfo.h>
#include "../devices/peripheral_service/McuManager.h"
#include <sstream>

// 
#define QINIU_BUCKET_NAME "http://video.ubtrobot.com/"

#define DIAGNOSTICS_TAG           "Diagnostics->"

boost::mutex Diagnostics::mutex_;
Diagnostics* Diagnostics::diagnostics_ = NULL;

class UploadLogFile: public FileObject {
  virtual void notify(TransferStates state, std::string msg = "", int code = 0)  {
    this->state = state;
  }

  public:
    UploadLogFile(): state(TRANSFER_FILE_STARTED) { }
    TransferStates state;
};

Diagnostics::Diagnostics()
    : log_thread_(NULL)
    , pDiagSpinTimer(nullptr)
    , diag_spin_called_time_(0)
    , wait_can_send_(false)
    , battery_check_time_last_(0.0)
    , log_uploading_(false)
    , level_trig_once_flg_(1)
    , battery_level_last_(-1)
    , bumper_state_last_(-1)
    , button_state_last_(-1)
    , system_check_cnt_(0)
    , isUpgrading(false)
    , battery_temp_last_(0)
    , imu_recv_time_last_(0)
    , imu_error_(-1)
    , odom_recv_time_last_(0)
    , gps_recv_time_last_(0)
    , odom_error_(1)
    , bms_info_recv_last_(-1)
    , bms_warning_stats_(-1)
    , chassis_warning_stats_(-1)
    , chassis_lf_recv_last_(-1)
    , chassis_lr_recv_last_(-1)
    , chassis_rf_recv_last_(-1)
    , chassis_rr_recv_last_(-1)
    , chassis_lf_error_stats_(-1)
    , chassis_rf_error_stats_(-1)
    , chassis_lr_error_stats_(-1)
    , chassis_rr_error_stats_(-1)
    , robot_run_mode_(0)
    , middle_fan_enable_(true)
    , robot_collision_(false)
    , voip_calling_status_(0)
    , data_base_init_ok_(false)
    , event_data_base_init_ok_(false)
    , chassis_upload_log_finish_(false)
    , monitor_upload_log_finish_(false)
    , mode_priority_thread_exit_(false)
    , robot_pose_sub_(TOPIC_ODOM, &Diagnostics::on_receive_odom, this)
    , chassis_diag_sub_(TOPIC_CHASSIS_DIAGNOSTICS, &Diagnostics::on_recv_chassis_diag_info, this)
    , localization_lost_stats_(0)
    , signal_bad_stats_(0)
    , ptz_error_stats_(0)
    , lidar_error_stats_(0)
    , ultra_error_stats_(0)
    , ads_error_stats_(0)
    , imu_error_stats_(0)
    , ccb_error_stats_(0)
    , sys_error_stats_(0)
    , chassis_error_stats_(0)
    , battery_error_stats_(0)
    , speaker_error_stats_(0)
    , mic_error_stats_(0)
    , voip_error_stats_(0)
    , flood_warning_stats_(0)
    , overtemp_warning_stats_(0)
    , emerge_stop_warning_stats_(0)
    , collision_warning_stats_(0)
    , abnormal_shutdown_warning_stats_(0)
    , mqtt_connect_state_(0)
    , event_store_num_(0)
    , system_delay_check_(false)
    , bms_communication_ok_(-1)
    , isBmsCharging_(false)
    , machine_status_(0)
    , ptz_serial_num_("")
    , chassis_serial_num_("")
    , main_bms_serial_num_("")
    , ccb_serial_num_("")
    , sys_serial_num_("")
    , simulation_event_switch_(0)
    {
    log_info(DIAGNOSTICS_TAG"%s", __FUNCTION__);
    int iRet = -1;
    
    aisound_tts_pub_ = nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);
    shttpd_diag_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SHTTPD_DIAGNOSTIC_MESSAGE, 100);
    power_off_pub_ = nh_.advertise<std_msgs::Int32>(TOPIC_POWER_OFF_MESSAGE, 100);
    robot_braked_pub_ = nh_.advertise<atris_msgs::RobotBraked>(TOPIC_ROBOT_BRAKED_MESSAGE, 100);
    reboot_braked_pub_ = nh_.advertise<atris_msgs::RebootBraked>(TOPIC_REBOOT_BRAKED_MESSAGE, 100);
    protect_mode_pub_ = nh_.advertise<atris_msgs::ProtectMode>(TOPIC_PROTECT_MODE_MESSAGE, 100);
    send_can_cmd_pub_ = nh_.advertise<atris_msgs::CanData>(TOPIC_CAN_CMD_MESSAGE, 100);
    mqtt_signal_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_RESPONSE_MESSAGE, 100);
    recharge_battery_pub_ = nh_.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_REQUEST_MESSAGE, 100);
    middle_fan_ctrl_pub_ = nh_.advertise<atris_msgs::MiddleFanControl>(TOPIC_MIDDLE_FAN_CMD, 100);
    mcu_upload_log_req_pub_ = nh_.advertise<atris_msgs::McuLogCtrl>(TOPIC_DIAG_MCU_LOG_MESSAGE_REQ,100);

    signal_req_sub_ = nh_.subscribe(TOPIC_SIGNAL_REQUEST_MESSAGE, 100, &Diagnostics::messageInstantReceive, this);
    signal_resp_sub_ = nh_.subscribe(TOPIC_SIGNAL_RESPONSE_MESSAGE, 100, &Diagnostics::notifyRobotControlStatus, this);
    diag_info_sub_ = nh_.subscribe(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100, &Diagnostics::robotInfoTopicCb, this);
    imu_info_sub_ = nh_.subscribe(TOPIC_IMU, 100, &Diagnostics::imuInfoTopicCb, this);
    robot_run_mode_sub_ = nh_.subscribe(TOPIC_ROBOT_RUN_MODE, 100, &Diagnostics::robotRunModeInfoCb, this);
    mqtt_connect_state_sub_ = nh_.subscribe(TOPIC_NETWORK_STATE, 100, &Diagnostics::robotConnectStateCb, this);
    tinyros::nh()->subscribe(chassis_diag_sub_);
    tinyros::nh()->subscribe(robot_pose_sub_);
    
    upgrade_status_sub_ = nh_.subscribe(TOPIC_SWUPGRADE_STATUS, 100, &Diagnostics::on_recv_upgrade_stats, this);
    mcu_upload_log_resp_sub_ = nh_.subscribe(TOPIC_DIAG_MCU_LOG_MESSAGE_RESP, 100, &Diagnostics::on_recv_mcu_upload_log_status, this);
    get_lamp_status_srv_client_ = nh_.serviceClient<atris_msgs::GetLampStatus>(SRV_GET_LAMP_STATUS);
    peripheral_ctrl_client_ = nh_.serviceClient<atris_msgs::PeripheralControl>(SRV_PERIPHERAL_CONTROL);
    get_standby_mode_srv_client_ = nh_.serviceClient<atris_msgs::GetStandbyMode>(SRC_GET_STANDBY_MODE);
    get_ppplaying_status_srv_client_ = nh_.serviceClient<atris_msgs::GetPPPlayingStatus>(SRV_GET_PPPLAYING_STATUS);
    get_disperse_status_srv_client_ = nh_.serviceClient<atris_msgs::GetDisperseStatus>(SRV_GET_DISPERSE_STATUS);
    get_patrol_status_srv_client_ = nh_.serviceClient<atris_msgs::GetPatrolStatus>(SRV_GET_PATROL_STATUS);
    get_ptz_status_srv_client_ = nh_.serviceClient<atris_msgs::GetPtzStatus>(SRV_GET_PTZ_STATUS);
    get_sw_version_srv_client_ = nh_.serviceClient<atris_msgs::GetSwVersion>(SRV_GET_SW_VERSION);
    get_chassis_info_srv_client_ = nh_.serviceClient<atris_msgs::GetChassisInfo>(SRV_GET_CHASSIS_INFO);
    get_ota_pre_condition_srv_ = nh_.advertiseService(SRV_GET_OTA_PRE_CONDITION, &Diagnostics::doGetOtaPreCondition, this);
    diag_info_srv_ = nh_.advertiseService(SRV_GET_DIAGNOSTICS_INFO, &Diagnostics::doGetDiagnosticsInfo, this);
    
    //low_battery_warn_level_ = Config::get_instance()->low_battery_warn;
    //log_debug("%s low battery warn level : %d",__FUNCTION__, low_battery_warn_level_);
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);

    utils = Utils::get_instance();
    // init run mode status
    memset(run_mode_status_, 0x00, sizeof(run_mode_status_));
        
    {
        // chassis_driver type
#ifdef _CHASSIS_MARSHELL_
        diag_["robot_info"]["chassis_driver"]["type"] = "M";
#else
        diag_["robot_info"]["chassis_driver"]["type"] = "J";
#endif
        diag_["robot_info"]["power"]["imx"] = 1;
        diag_["robot_info"]["main_lidar"]["error"] = -1;
        diag_["robot_info"]["slave_lidar"]["error"] = -1;
        diag_["robot_info"]["ultrasound"]["error"] = -1;
        diag_["robot_info"]["gyro"]["error"] = -1;
        diag_["robot_info"]["gps"]["error"] = -1;
        diag_["robot_info"]["gps"]["lati"] = 0.0;
        diag_["robot_info"]["gps"]["long"] = 0.0;

        // added initial value for shadow
        initDiagAttr();

    }

    std::thread repor_event_thd(&Diagnostics::reportItvEventThread, this);
    repor_event_thd.detach();

    std::thread delay_thread(&Diagnostics::storeDiagEventThread, this);
    delay_thread.detach();

    pDiagSpinTimer = new CTimer("diag_spin_timer");
    pDiagSpinTimer->AsyncLoop(10 * 1000, &Diagnostics::diagUdpSpin, this);// async executed loop, here the time is set to 5 sec

    // self check timer
    // 3 miutes later to check ptz error status to wait ptz system up
    pSelfDiagTimer = new CTimer("self_check_diag_timer");
    pSelfDiagTimer->AsyncOnce(3* 60 * 1000, &Diagnostics::sysDiagDelayProc, this); // wait 3 minutes to do the system diag check for ptz device

    // init diag database
    data_base_init_ok_ = initDiagDataBase();
    if(data_base_init_ok_)
    {
        log_info("%s diagnostics module , database init ok...",__FUNCTION__);
    }
    else
    {
        log_error("%s diagnostics module , database init failed!!!",__FUNCTION__);
    }

    event_data_base_init_ok_ = initDiagEventStoreDataBase();
    if(event_data_base_init_ok_)
    {
        log_info("%s diagnostics module , event store database init ok...",__FUNCTION__);
    }
    else
    {
        log_error("%s diagnostics module , event store database init failed!!!",__FUNCTION__);
    }


    #if 0
    iRet = initEventVarStats(); // this function is used if we report event occured to web more than once
    if(iRet < 0)
    {
        log_error("init event variable status failed");
    }
    else
    {
        log_info("init event variable status success...");
    }
    #endif

    // we use a different thread to insert the event into the data base one by one to reduce the recv event latency
    std::thread thd(&Diagnostics::processDiagEvent, this); // database
    thd.detach();

    std::thread check_device_error_thd(&Diagnostics::checkDevStatus, this);
    check_device_error_thd.detach();

    std::thread check_run_mode_thd(&Diagnostics::run_mode_thread, this);
    check_run_mode_thd.detach();
 }

Diagnostics::~Diagnostics() {
}

Diagnostics* Diagnostics::get_instance() {
    if (!diagnostics_) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        if (!diagnostics_) {
            diagnostics_ = new Diagnostics();
        }
    }
    return diagnostics_;
}

Json::Value Diagnostics::getValue() const {
    boost::unique_lock<boost::mutex> lock(mutex_);
    return diag_;
}

void Diagnostics::initDiagAttr(void)
{
    log_info("%s",__FUNCTION__);
    shm::Robot shmrbt;
    shm::CompanyId shmcomid;
    shm::iMemory_read_Robot(&shmrbt);
    shm::iMemory_read_CompanyId(&shmcomid);

    diag_["robot_info"]["base"]["binded"] = "";
    diag_["robot_info"]["base"]["company"] = "";
    diag_["robot_info"]["base"]["sn"] = shmrbt.robot.sn;
    diag_["robot_info"]["base"]["run_mode"] = 0;
    diag_["robot_info"]["base"]["work_time"] = 0.0f;
    diag_["robot_info"]["base"]["machine_status"] = 0;

    diag_["robot_info"]["4g"]["error"] = 0;
    diag_["robot_info"]["4g"]["level"] = 0;
    diag_["robot_info"]["doublecom"]["level"] = 0;

    diag_["robot_info"]["navigation"]["navtype"] = 1;
    diag_["robot_info"]["navigation"]["ao_mode"] = 1;

    diag_["robot_info"]["battery"]["alarm"] = 0;
    diag_["robot_info"]["battery"]["bat_num"] = 0;
    diag_["robot_info"]["battery"]["charge_cnt"] = 0;
    diag_["robot_info"]["battery"]["cstatus"] = 0;
    diag_["robot_info"]["battery"]["current"] = 0;
    diag_["robot_info"]["battery"]["discharge_cnt"] = 0;
    diag_["robot_info"]["battery"]["health"] = 0;
    diag_["robot_info"]["battery"]["level"] = 20;
    diag_["robot_info"]["battery"]["relay_status"] = 0;
    diag_["robot_info"]["battery"]["status"] = 0;
    diag_["robot_info"]["battery"]["temp_max"] = 0;
    diag_["robot_info"]["battery"]["temp_min"] = 0;
    diag_["robot_info"]["battery"]["tstatus"] = 0;
    diag_["robot_info"]["battery"]["voltage"] = 0;
    diag_["robot_info"]["battery"]["vstatus"] = 0;
    diag_["robot_info"]["battery"]["error"] = 0;

    diag_["robot_info"]["brake"]["button"] = 0;
    diag_["robot_info"]["brake"]["charge"] = 0;
    diag_["robot_info"]["brake"]["charge_bumper"] = 0;
    diag_["robot_info"]["brake"]["front_bumper"] = 0;
    diag_["robot_info"]["brake"]["tear_bumper"] = 0;
    diag_["robot_info"]["brake"]["front_bumper_type"] = "";
    diag_["robot_info"]["brake"]["tear_bumper_type"] = "";
    diag_["robot_info"]["control"]["CMotor_brake"] = 0;
    diag_["robot_info"]["control"]["CEmergency_brake"] = 0;
    diag_["robot_info"]["control"]["W_software_brake"] = 0;

    diag_["robot_info"]["source"]["reboot"] = 0;
    diag_["robot_info"]["source"]["front_bumper"] = 0;
    diag_["robot_info"]["source"]["tear_bumper"] = 0;
    diag_["robot_info"]["source"]["charge_bumper"] = 0;
    diag_["robot_info"]["source"]["charge"] = 0;
    diag_["robot_info"]["source"]["can"] = 0;
    diag_["robot_info"]["source"]["iap"] = 0;
    diag_["robot_info"]["source"]["button"] = 0;

    diag_["robot_info"]["camera"]["nvr_ip"] = "";
    diag_["robot_info"]["camera"]["ptz_ip"] = "";
    diag_["robot_info"]["camera"]["pan_angle"] = 0.0f;
    diag_["robot_info"]["camera"]["tilt_angle"] = 0.0f;
    diag_["robot_info"]["camera"]["zoom_value"] = 0.0f;
    diag_["robot_info"]["camera"]["light_status"] = 0;
    diag_["robot_info"]["camera"]["wiper_status"] = 0;
    diag_["robot_info"]["camera"]["status"] = 0;
    diag_["robot_info"]["camera"]["error"] = 0;

    diag_["robot_info"]["charge"]["bluetooth"] = 0;
    diag_["robot_info"]["charge"]["electrodes_status"] = 0;
    diag_["robot_info"]["charge"]["electrodes_voltage"] = 0;
    diag_["robot_info"]["charge"]["switch"] = 0;

    diag_["robot_info"]["chassis_driver"]["bat_voltage"] = 0;
    diag_["robot_info"]["chassis_driver"]["bat_current"] = 0;
    diag_["robot_info"]["chassis_driver"]["error"] = -1;
    diag_["robot_info"]["chassis_driver"]["motor_current"] = 0;
    diag_["robot_info"]["chassis_driver"]["temp_ic"] = 0;
    diag_["robot_info"]["chassis_driver"]["temp_motor_left"] = 0;
    diag_["robot_info"]["chassis_driver"]["temp_motor_right"] = 0;

    diag_["robot_info"]["disperse"]["status"] = 0;

    diag_["robot_info"]["fan"]["bottom"]["error"] = 0;
    diag_["robot_info"]["fan"]["bottom"]["speed_in"] = 0;
    diag_["robot_info"]["fan"]["bottom"]["speed_out"] = 0;

    diag_["robot_info"]["fan"]["middle"]["error"] = 0;
    diag_["robot_info"]["fan"]["middle"]["speed_in"] = 0;
    diag_["robot_info"]["fan"]["middle"]["speed_out"] = 0;

    diag_["robot_info"]["imu"]["error"] = -1;
    diag_["robot_info"]["ads"]["error"] = 0;
    diag_["robot_info"]["gyro"]["error"] = 0;
    diag_["robot_info"]["intercom"]["status"] = 0;
    diag_["robot_info"]["light"]["rb_status"] = 0;
    diag_["robot_info"]["light"]["w_status"] = 0;

    diag_["robot_info"]["main_lidar"]["error"] = 0;

    diag_["robot_info"]["odom"]["error"] = 0;
    diag_["robot_info"]["odom"]["odo"] = 0.0f;
    diag_["robot_info"]["odom"]["speed_linear"] = 0.0f;
    diag_["robot_info"]["odom"]["speed_theta"] = 0.0f;

    diag_["robot_info"]["patrol"]["status"] = 0;

    diag_["robot_info"]["power"]["disperse"] = 0;
    diag_["robot_info"]["power"]["fan_in"] = 0;
    diag_["robot_info"]["power"]["fan_out"] = 0;
    diag_["robot_info"]["power"]["gps"] = 0;
    diag_["robot_info"]["power"]["imx"] = 0;
    diag_["robot_info"]["power"]["ks106"] = 0;
    diag_["robot_info"]["power"]["ks136"] = 0;
    diag_["robot_info"]["power"]["m_fan_in"] = 0;
    diag_["robot_info"]["power"]["m_fan_out"] = 0;
    diag_["robot_info"]["power"]["main_lidar"] = 0;
    diag_["robot_info"]["power"]["netswitch"] = 0;
    diag_["robot_info"]["power"]["slave_lidar"] = 0;
    diag_["robot_info"]["power"]["yuntai"] = 0;

    diag_["robot_info"]["ppplay"]["duration"] = 0;
    diag_["robot_info"]["ppplay"]["interval"] = 0;
    diag_["robot_info"]["ppplay"]["name"] = "";
    diag_["robot_info"]["ppplay"]["pts"] = 0;
    diag_["robot_info"]["ppplay"]["status"] = 0;

    diag_["robot_info"]["sensor_hall"]["hall1"] = 0;
    diag_["robot_info"]["sensor_hall"]["hall2"] = 0;
    diag_["robot_info"]["sensor_hall"]["hall3"] = 0;

    diag_["robot_info"]["sensor_liquid"]["status"] = 0;
    diag_["robot_info"]["slave_lidar"]["error"] = 0;

    diag_["robot_info"]["speed"]["value"] = 0.0f;

    diag_["robot_info"]["temp_humi"]["gaussian"] = 0.0f;
    diag_["robot_info"]["temp_humi"]["humi_env"] = 0.0f;
    diag_["robot_info"]["temp_humi"]["imx"] = 0.0f;
    diag_["robot_info"]["temp_humi"]["kuangshi"] = 0.0f;
    diag_["robot_info"]["temp_humi"]["motor_left"] = 0;
    diag_["robot_info"]["temp_humi"]["motor_right"] = 0;
    diag_["robot_info"]["temp_humi"]["temp_env"] = 0;

    Json::Value tmp(0);
    diag_["robot_info"]["ultrasound"]["data"].append(tmp);
    diag_["robot_info"]["ultrasound"]["error"] = 1;
    diag_["robot_info"]["upgrade"]["status"] = 0;

    diag_["robot_info"]["version"]["battery_monitor"] = "";
    diag_["robot_info"]["version"]["bms"] = "";
    diag_["robot_info"]["version"]["gs"] = "";
    diag_["robot_info"]["version"]["imx"] = "";
    diag_["robot_info"]["version"]["power"] = "";

    diag_["robot_info"]["volume"]["muted"] = 0;
    diag_["robot_info"]["volume"]["gs"] = 0;

    diag_["robot_info"]["voip"]["switch"] = 0;
    diag_["robot_info"]["voip"]["volume"] = 1;
}

int Diagnostics::getBatLevel()
{
    int iRet = -1;
    int bat_level_tmp;
    Json::Value tmp = getValue();

    if(!tmp["robot_info"].isNull() && !tmp["robot_info"]["battery"].isNull() && !tmp["robot_info"]["battery"]["level"].isNull())
    {
      bat_level_tmp = tmp["robot_info"]["battery"]["level"].asInt();
      // check if the bat data is valid
      if(bat_level_tmp >= 0 && bat_level_tmp <= 100)
      {
          //log_info("get bat level success bat level = %d\r\n",bat_level_tmp);
          iRet = bat_level_tmp;
      }
      else
      {
          log_error("battery_level_data invalid!!! bat level recv = %d\r\n",bat_level_tmp);
          return iRet;
      }
    }

    return iRet;
}

bool Diagnostics::checkRebootBrake() {
    Json::Value tmp = diag_;
    if (!tmp["robot_info"].isNull() 
        && !tmp["robot_info"]["brake"].isNull() 
        && !tmp["robot_info"]["brake"]["source"].isNull()) {
        if (!tmp["robot_info"]["brake"]["source"]["button"].isNull()) {
           if (tmp["robot_info"]["brake"]["source"]["button"].asInt()  > 0){
                return false;
           }
        }
        if (!tmp["robot_info"]["brake"]["source"]["charge"].isNull()) {
           if (tmp["robot_info"]["brake"]["source"]["charge"].asInt()  > 0){
               return false;
           }
        }
        if (!tmp["robot_info"]["brake"]["source"]["charge_bumper"].isNull()) {
           if (tmp["robot_info"]["brake"]["source"]["charge_bumper"].asInt()  > 0){
               return false;
           }
        }
        if (!tmp["robot_info"]["brake"]["source"]["front_bumper"].isNull()) {
           if (tmp["robot_info"]["brake"]["source"]["front_bumper"].asInt()  > 0){
               return false;
           }
        }
        if (!tmp["robot_info"]["brake"]["source"]["tear_bumper"].isNull()) {
           if (tmp["robot_info"]["brake"]["source"]["tear_bumper"].asInt()  > 0){
               return false;
           }
        }


        if (!tmp["robot_info"]["brake"]["source"]["can"].isNull()) {
           if (tmp["robot_info"]["brake"]["source"]["can"].asInt()  > 0){
               return false;
           }
        }
        if (!tmp["robot_info"]["brake"]["source"]["iap"].isNull()) {
            if (tmp["robot_info"]["brake"]["source"]["iap"].asInt()  > 0){
                return false;
            }
        }
        if (!tmp["robot_info"]["brake"]["source"]["reboot"].isNull()) {
            if (tmp["robot_info"]["brake"]["source"]["reboot"].asInt()  > 0) {
                return true;
            }
        }
    }
    return false;           
}

bool Diagnostics::checkRobotBrake() {
    Json::Value tmp = diag_;
    if (!tmp["robot_info"].isNull() 
        && !tmp["robot_info"]["brake"].isNull()
        && !tmp["robot_info"]["brake"]["control"].isNull()) {
        if (!tmp["robot_info"]["brake"]["control"]["CMotor_brake"].isNull()) {
            if (tmp["robot_info"]["brake"]["control"]["CMotor_brake"].asInt() > 0) {
                return true;
            }
        }
        if (!tmp["robot_info"]["brake"]["control"]["CEmergency_brake"].isNull()) {
            if (tmp["robot_info"]["brake"]["control"]["CEmergency_brake"].asInt() > 0) {
                return true;
            }
        }
        if (!tmp["robot_info"]["brake"]["control"]["W_software_brake"].isNull()) {
            if (tmp["robot_info"]["brake"]["control"]["W_software_brake"].asInt() > 0) {
                return true;
            }
        }
        if (!tmp["robot_info"]["brake"]["control"]["button_hardware_brake"].isNull()) {
            if (tmp["robot_info"]["brake"]["control"]["button_hardware_brake"].asInt() > 0) {
                return true;
            }
        }
    }
    return false;
}

void Diagnostics::checkBatteryLevel(int level) 
{
    double now = tinyros::Time().now().toSec();

    if(level < 0 || level > 100)
    {
        log_error("%s battery level invalid , level : %d",__FUNCTION__, level);
        return;
    }
    
    //Json::Value tmp = getValue();

    //int level = 100;
    int bat_level_tmp;
    int ext_low_bat_trig_ = 0;

    static int bat_reduce_low_flag_ = 1;
    static int bat_level_low_15_flag = 1;
    static int bat_level_low_10_flag = 1;
    static int bat_level_low_5_flag = 1;
    static int bat_level_low_0_flag = 1;

    char rand_str[16] = {0};

    checkIfNeedReportStartUp(level);
    #if 0
    if(!tmp["robot_info"].isNull() && !tmp["robot_info"]["battery"].isNull() && !tmp["robot_info"]["battery"]["level"].isNull())
    {
        bat_level_tmp = tmp["robot_info"]["battery"]["level"].asInt();
        // check if the bat data is valid
        if(bat_level_tmp >= 0 && bat_level_tmp <= 100)
        {
            level = bat_level_tmp;
        }
        else
        {
            log_error("battery_level_data invalid!!! bat level recv = %d\r\n",bat_level_tmp);
            return;
        }
    }
    else
    {
        // information not complete just return
        log_warn("battery info not complete!!! just return\r\n");
        return;
    }
    #endif

    shm::Robot shm_robot;
    int low_power_warn_val;

    iMemory_read_Robot(&shm_robot);

    //log_info("%s battery reduce low flag : %d , battery level last : %d , battery level : %d low battery warn value = %d",__FUNCTION__, bat_reduce_low_flag_,battery_level_last_, level, shm_robot.appdata.power_warning_value);
    low_power_warn_val = shm_robot.appdata.power_warning_value;

   // log_debug("auto charge battery:%d,level:%d", Config::get_instance()->nav_auto_charge_battery, level);
    if (Config::get_instance()->nav_auto_charge_battery > level
        || (shm_robot.appdata.power_warning_charge > level)) {
        Json::Value js_req;
        Json::FastWriter fw;
        js_req["title"] = "request_recharge_battery";
        js_req["content"]["battery"] = level;
        atris_msgs::SignalMessage req;
        std::string js_data = fw.write(js_req);
        req.title = "request_recharge_battery";
        req.msg = js_data.c_str();
        recharge_battery_pub_.publish(req);
    }

    // bat level equal to 30 when power on or bat level reduce to 30 percent, sound once low battery warning
    if(low_power_warn_val > 15)
    {
        if((battery_level_last_ > level) && (level == low_power_warn_val))
        {

            if(bat_reduce_low_flag_)
            {
                bat_reduce_low_flag_ = 0;
                std::string battery_low_normal_serial_num = "";
                Utils::get_instance()->getRandStr(rand_str);
                battery_low_normal_serial_num = rand_str;
                notifyBatteryWarningEvent("battery_level_low_normal", (uint64_t)now, battery_low_normal_serial_num);
                log_warn("battery reduce to low power threshold %d percent , level now = %d , battery level last round = %d , report it to web", low_power_warn_val, level , battery_level_last_);
            }
        }

        if(!bat_reduce_low_flag_ && ((battery_level_last_ < level)&&(level == (low_power_warn_val + 1))))
        {
            log_info("battery level recover to more than 40 percent\r\n");
            bat_reduce_low_flag_ = 1;
        }
    }

    if(((battery_level_last_ > level) && (level == 15)))
    {
        if(bat_level_low_15_flag)
        {
            log_info("battery level reduce to or equal to 15 percent, level : %d, battery level last : %d\r\n", level, battery_level_last_);
            reportBatteryLevelLowUrgent();
            bat_level_low_15_flag = 0;
        }
    }
    else if(((battery_level_last_ > level) && (level == 10)))
    {
        if(bat_level_low_10_flag)
        {
            log_info("battery level reduce to or equal to 10 percent, level : %d, battery level last : %d\r\n", level, battery_level_last_);
            reportBatteryLevelLowUrgent();
            bat_level_low_10_flag = 0;
        }
    }
    else if(((battery_level_last_ > level) && (level == 5)))
    {
        if(bat_level_low_5_flag)
        {
            log_info("battery level reduce to or equal to 5 percent, level : %d, battery level last : %d\r\n", level, battery_level_last_);
            reportBatteryLevelLowUrgent();
            bat_level_low_5_flag = 0;
        }
    }
    else if(((battery_level_last_ > level) && (level == 0)))
    {
        if(bat_level_low_0_flag)
        {
            log_info("battery level reduce to or equal to 0 percent, level : %d, battery level last : %d\r\n", level, battery_level_last_);
            reportBatteryLevelLowUrgent();
            bat_level_low_0_flag = 0;
        }
    }
    
    if(battery_level_last_ < level)
    {
        log_info("battery charging");
        if(!bat_level_low_15_flag && (level == 16))
        {
            log_info("battery level recover to more than 15 percent\r\n");
            bat_level_low_15_flag = 1;
        }
        else if(!bat_level_low_10_flag && (level == 11))
        {
            log_info("battery level recover to more than 10 percent\r\n");
            bat_level_low_10_flag = 1;
        }
        else if(!bat_level_low_5_flag && (level == 6))
        {
            log_info("battery level recover to more than 5 percent\r\n");
            bat_level_low_5_flag = 1;
        }
        else if(!bat_level_low_0_flag && (level == 1))
        {
            log_info("battery level recover to more than 0 percent\r\n");
            bat_level_low_0_flag = 1;
        }
    }

    battery_level_last_ = level;
}

// report to web if the program start up and battery level reaches the threshold value
void Diagnostics::checkIfNeedReportStartUp(int bat_level)
{
    static int check_battery_level_flag = 0;

    if(!check_battery_level_flag)
    {
        check_battery_level_flag = 1;
        if(bat_level == Config::get_instance()->low_battery_warn)
        {
            reportBatteryLevelLowNormal();
        }
        else if(bat_level == 15 || bat_level == 10 || bat_level == 5 || bat_level == 0)
        {
            reportBatteryLevelLowUrgent();
        }
    }
}

void Diagnostics::reportBatteryLevelLowNormal(void)
{
    double now = tinyros::Time().now().toSec();
    char rand_str[16] = {0};
    std::string battery_low_normal_serial_num = "";

    Utils::get_instance()->getRandStr(rand_str);
    battery_low_normal_serial_num = rand_str;
    notifyBatteryWarningEvent("battery_level_low_normal", (uint64_t)now, battery_low_normal_serial_num);
}

void Diagnostics::reportBatteryLevelLowUrgent(void)
{
    double now = tinyros::Time().now().toSec();
    char rand_str[16] = {0};
    std::string battery_low_urgent_serial_num = "";

    Utils::get_instance()->getRandStr(rand_str);
    battery_low_urgent_serial_num = rand_str;
    notifyBatteryWarningEvent("battery_level_low_urgent", (uint64_t)now, battery_low_urgent_serial_num);
}

void Diagnostics::diagUdpSpin(void) 
{
    //bool isSpin = true;
    long long diag_spin_called_time_this;
    diag_spin_called_time_this = __getmstime();
    diag_spin_called_time_  = diag_spin_called_time_this;

    struct timeval start,end;
    float duration;

    gettimeofday(&start,NULL);
    // check battery level
    //checkBatteryLevel();
       
    try { // notify_diagnostics_robot_info
        Config *cfg = Config::get_instance();
        if (!cfg->local_ip.empty()) {
            Json::FastWriter jwriter;
            Json::Value root;
            ros::Time now = ros::Time::now();
            std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
            root["title"] = "notify_robot_info";
            root["content"] = diagnosticsRobotInfo(uid.str(), (uint64_t) (now.toSec() * 1000000000ull));

            shm::Robot shmrbt;
            shm::CompanyId shmcomid;
            shm::iMemory_read_Robot(&shmrbt);
            shm::iMemory_read_CompanyId(&shmcomid);

            /*************************/
            // To fixed old kuangshi
            std::string company_id = shmcomid.company_id;
            root["content"]["robot"]["sn"] = shmrbt.robot.sn;
            root["content"]["robot"]["power"] = "on";
            root["content"]["robot"]["company"] = company_id.empty() ? "UBT_ATRIS" : company_id;
            root["content"]["gps_info"]["state"] = -1;
            root["content"]["gps_info"]["lati"] = "0";
            root["content"]["gps_info"]["long"] = "0";

            Json::Value tmp = getValue();
            if (!tmp["robot_info"].isNull() && !tmp["robot_info"]["gps"].isNull()) {
                if (!tmp["robot_info"]["gps"]["error"].isNull()) {
                    root["content"]["gps_info"]["state"] = tmp["robot_info"]["gps"]["error"].asInt();
                }
                if (!tmp["robot_info"]["gps"]["lati"].isNull()) {
                    root["content"]["gps_info"]["lati"] = tmp["robot_info"]["gps"]["lati"].asString();
                }
                if (!tmp["robot_info"]["gps"]["long"].isNull()) {
                    root["content"]["gps_info"]["long"] = tmp["robot_info"]["gps"]["long"].asString();
                }
                
                if (!tmp["robot_info"]["gps"]["alti"].isNull()) {
                    root["content"]["gps_info"]["alti"] = tmp["robot_info"]["gps"]["alti"];
                }
            }
            
            if (!tmp["robot_info"]["power"].isNull() && !tmp["robot_info"]["power"]["imx"].isNull()) {
                root["content"]["robot"]["power"] = (tmp["robot_info"]["power"]["imx"].asInt() != 0) ? "on" : "off";
            }
            /*************************/

            std::string json = jwriter.write(root);
            std::vector<uint8_t> buffer(json.size() + 1, 0);
            snprintf((char*)&buffer.at(0), buffer.size(), json.c_str(), json.size());
            udp::endpoint endpoint(boost::asio::ip::address_v4::broadcast(), 60000);
            udp::socket socket(io_service_, udp::endpoint(udp::v4(),0));
            socket.set_option(boost::asio::socket_base::broadcast(true));
            socket.set_option(boost::asio::socket_base::reuse_address(true));
            socket.async_send_to(boost::asio::buffer(buffer), endpoint, boost::bind(&Diagnostics::handleSend, this, boost::asio::placeholders::error));

            // Record diagnostics
            log_diag("%s", json.c_str());

            // publish to rosshttp
            atris_msgs::SignalMessage diag_msg;
            diag_msg.msg = json;
            diag_msg.type ="shttpd";
            shttpd_diag_pub_.publish(diag_msg);

            diag_msg.type = "local_mqtt";
            diag_msg.account = shmrbt.robot.receiver;
            Utils::get_instance()->responseResult(diag_msg, root["content"], "notify_robot_info");

            Json::Value remote_mqtt_info;

            remote_mqtt_info["domain"] = "setting";
            remote_mqtt_info["productName"] = "Atris";
            remote_mqtt_info["sn"] = shmrbt.robot.sn;
            remote_mqtt_info["state"]["reported"] = root["content"];
            remote_mqtt_info["type"] = "update";
            remote_mqtt_info["title"] = "notify_robot_info";

            diag_msg.type = "remote_mqtt";
            diag_msg.account = shmrbt.robot.receiver;
            diag_msg.msgID = uid.str();
            diag_msg.title = "notify_robot_info";
            diag_msg.msg = jwriter.write(remote_mqtt_info);
            log_info("%s notify robot info to shadow",__FUNCTION__);
            // for debug use
            //log_info("shadow msg : %s",diag_msg.msg.c_str());
            mqtt_signal_pub_.publish(diag_msg);
       }
    } catch(std::runtime_error& e) { 
       log_error("%s %s", __FUNCTION__, e.what());
    }

    // post data to remote server every 5 secs
    gettimeofday(&end,NULL);
    duration = (end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
    //log_info("%s function consume duration : %.10f usec\n",__FUNCTION__, duration);

}

void Diagnostics::handleSend(const boost::system::error_code& error) {
    if (error) {
        if (error == boost::system::errc::io_error) {
            log_error("%s Socket write operation returned IO error.", __FUNCTION__);
        } else if (error == boost::system::errc::no_such_device) {
            log_error("%s Socket write operation returned no device.", __FUNCTION__);
        } else {
            log_error("%s Unknown error returned during write operation.",__FUNCTION__);
        }
    }
}

// recv imu published info , if we cannot read imu info for a period of time, set imu error to 0
void Diagnostics::imuInfoTopicCb (const atris_msgs::RobotImu& msg)
{

    struct timeval imu_recv_time_this;
    gettimeofday(&imu_recv_time_this,NULL);

    imu_recv_time_last_ = imu_recv_time_this.tv_sec;
}

// receive odom info from odom topic
void Diagnostics::on_receive_odom(const tinyros::atris_msgs::RobotPose& msg)
{
    struct timeval odom_recv_time_this;
    gettimeofday(&odom_recv_time_this, NULL);
    //int odom_error_this;

    odom_topic_info.pos_x = msg.p_x;
    odom_topic_info.pos_y = msg.p_y;
    odom_topic_info.pos_a = msg.p_z;
    odom_topic_info.speed_x = msg.v_x;
    odom_topic_info.speed_y = msg.v_y;
    odom_topic_info.speed_a = msg.v_z;
    odom_topic_info.dist = msg.odom;

    //log_info("speed a : %lf, v_x : %lf, v_y : %lf odom : %lf", msg.v_z, msg.v_x, msg.v_y, msg.odom);
    // detect odom communication error
    #if 0
    if((odom_recv_time_this.tv_sec - odom_recv_time_last_) > 5)
    {
        odom_error_ = 1;
    }
    else
    {
        odom_error_ = 0;
    }
    
    #endif

    odom_recv_time_last_ = odom_recv_time_this.tv_sec;
}

void Diagnostics::robotInfoTopicCb (const atris_msgs::RobotInfo& msg) {
    if (!msg.json.empty()) {
        Json::Reader reader;
        Json::Value root;
        reader.parse(msg.json, root);

        boost::unique_lock<boost::mutex> lock(mutex_);
        if (!root["robot_info"].isNull()) {
            if (!root["robot_info"]["base"].isNull()) {
                if (!root["robot_info"]["base"]["sn"].isNull()) {
                    diag_["robot_info"]["base"]["sn"] = root["robot_info"]["base"]["sn"];
                }
                if (!root["robot_info"]["base"]["binded"].isNull()) {
                    diag_["robot_info"]["base"]["binded"] = root["robot_info"]["base"]["binded"];
                }

                if (!root["robot_info"]["base"]["machine_status"].isNull()) {
                    diag_["robot_info"]["base"]["machine_status"] = root["robot_info"]["base"]["machine_status"];

                    int machine_status_this = diag_["robot_info"]["base"]["machine_status"].asInt();

                    machine_status_ = diag_["robot_info"]["base"]["machine_status"].asInt();
                    //log_info("machine status : %d", machine_status_);
                    if(machine_status_this == 0 && machine_status_ == 1)
                    {
                        log_info("wake up from standby mode , wait 3 min to check ptz device");
                        system_delay_check_ = false;

                        if(pSelfDiagTimer != NULL)
                        {
                            //pSelfDiagTimer->DeleteThread();
                            delete pSelfDiagTimer;
                            pSelfDiagTimer = new CTimer("self_check_diag_timer");
                            pSelfDiagTimer->AsyncOnce(3* 60 * 1000, &Diagnostics::sysDiagDelayProc, this);
                        }
                    }

                    machine_status_ = machine_status_this;
                }
            }

            if (!root["robot_info"]["shaked"].isNull()) {
                if (!root["robot_info"]["shaked"]["value"].isNull()) {
                    diag_["robot_info"]["shaked"]["value"] = root["robot_info"]["shaked"]["value"];
                }
            }

            if (!root["robot_info"]["speed"].isNull()) {
                if (!root["robot_info"]["speed"]["value"].isNull()) {
                    diag_["robot_info"]["speed"]["value"] = root["robot_info"]["speed"]["value"];
                }
            }

            if (!root["robot_info"]["volume"].isNull()) {
                if (!root["robot_info"]["volume"]["muted"].isNull()) {
                    diag_["robot_info"]["volume"]["muted"] = root["robot_info"]["volume"]["muted"];
                }
                if (!root["robot_info"]["volume"]["value"].isNull()) {
                    diag_["robot_info"]["volume"]["value"] = root["robot_info"]["volume"]["value"];
                }
            }

            if (!root["robot_info"]["light"].isNull()) {
                if (!root["robot_info"]["light"]["w_status"].isNull()) {
                    diag_["robot_info"]["light"]["w_status"] = root["robot_info"]["light"]["w_status"];
                }
                if (!root["robot_info"]["light"]["rb_status"].isNull()) {
                    diag_["robot_info"]["light"]["rb_status"] = root["robot_info"]["light"]["rb_status"];
                }
            }

            if (!root["robot_info"]["ppplay"].isNull()) {
                if (!root["robot_info"]["ppplay"]["status"].isNull()) {
                    diag_["robot_info"]["ppplay"]["status"] = root["robot_info"]["ppplay"]["status"];
                }
                if (!root["robot_info"]["ppplay"]["name"].isNull()) {
                    diag_["robot_info"]["ppplay"]["name"] = root["robot_info"]["ppplay"]["name"];
                }
                if (!root["robot_info"]["ppplay"]["pts"].isNull()) {
                    diag_["robot_info"]["ppplay"]["pts"] = root["robot_info"]["ppplay"]["pts"];
                }
                if (!root["robot_info"]["ppplay"]["duration"].isNull()) {
                    diag_["robot_info"]["ppplay"]["duration"] = root["robot_info"]["ppplay"]["duration"];
                }
                if (!root["robot_info"]["ppplay"]["interval"].isNull()) {
                    diag_["robot_info"]["ppplay"]["interval"] = root["robot_info"]["ppplay"]["interval"];
                }
            }

            if (!root["robot_info"]["disperse"].isNull()) {
                if (!root["robot_info"]["disperse"]["status"].isNull()) {
                    diag_["robot_info"]["disperse"]["status"] = root["robot_info"]["disperse"]["status"];
                }
            }

            if (!root["robot_info"]["upgrade"].isNull()) {
                if (!root["robot_info"]["upgrade"]["status"].isNull()) {
                    diag_["robot_info"]["upgrade"]["status"] = root["robot_info"]["upgrade"]["status"];
                }
            }

            if (!root["robot_info"]["upgrade"].isNull()) {
                if (!root["robot_info"]["upgrade"]["upgrading"].isNull()) {
                    diag_["robot_info"]["upgrade"]["upgrading"] = root["robot_info"]["upgrade"]["upgrading"];
                }
            }

            if (!root["robot_info"]["patrol"].isNull()) {
                if (!root["robot_info"]["patrol"]["status"].isNull()) {
                    diag_["robot_info"]["patrol"]["status"] = root["robot_info"]["patrol"]["status"];
                }
            }

            if (!root["robot_info"]["camera"].isNull()) {
                if (!root["robot_info"]["camera"]["status"].isNull()) {
                    diag_["robot_info"]["camera"]["status"] = root["robot_info"]["camera"]["status"];
                }
                if (!root["robot_info"]["camera"]["ptz_ip"].isNull()) {
                    diag_["robot_info"]["camera"]["ptz_ip"] = root["robot_info"]["camera"]["ptz_ip"];
                }
                if (!root["robot_info"]["camera"]["nvr_ip"].isNull()) {
                    diag_["robot_info"]["camera"]["nvr_ip"] = root["robot_info"]["camera"]["nvr_ip"];
                }

                if (!root["robot_info"]["camera"]["pan_angle"].isNull()) {
                    diag_["robot_info"]["camera"]["pan_angle"] = root["robot_info"]["camera"]["pan_angle"];
                }
                if (!root["robot_info"]["camera"]["tilt_angle"].isNull()) {
                    diag_["robot_info"]["camera"]["tilt_angle"] = root["robot_info"]["camera"]["tilt_angle"];
                }
                if (!root["robot_info"]["camera"]["zoom_value"].isNull()) {
                    diag_["robot_info"]["camera"]["zoom_value"] = root["robot_info"]["camera"]["zoom_value"];
                }

                if (!root["robot_info"]["camera"]["light_status"].isNull()) {
                    diag_["robot_info"]["camera"]["light_status"] = root["robot_info"]["camera"]["light_status"];
                }
                if (!root["robot_info"]["camera"]["wiper_status"].isNull()) {
                    diag_["robot_info"]["camera"]["wiper_status"] = root["robot_info"]["camera"]["wiper_status"];
                }
                if (!root["robot_info"]["camera"]["error"].isNull()) {
                    diag_["robot_info"]["camera"]["error"] = root["robot_info"]["camera"]["error"];
                }

            }

            if (!root["robot_info"]["version"].isNull()) {
                if (!root["robot_info"]["version"]["imx"].isNull()) {
                    diag_["robot_info"]["version"]["imx"] = root["robot_info"]["version"]["imx"];
                }
                if (!root["robot_info"]["version"]["power"].isNull()) {
                    diag_["robot_info"]["version"]["power"] = root["robot_info"]["version"]["power"];
                }
                if (!root["robot_info"]["version"]["power_iap"].isNull()) {
                    diag_["robot_info"]["version"]["power_iap"] = root["robot_info"]["version"]["power_iap"];
                }
                if (!root["robot_info"]["version"]["gs"].isNull()) {
                    diag_["robot_info"]["version"]["gs"] = root["robot_info"]["version"]["gs"];
                }
                if (!root["robot_info"]["version"]["battery_monitor"].isNull()) {
                    diag_["robot_info"]["version"]["battery_monitor"] = root["robot_info"]["version"]["battery_monitor"];
                }
                if (!root["robot_info"]["version"]["bms"].isNull()) {
                    diag_["robot_info"]["version"]["bms"] = root["robot_info"]["version"]["bms"];
                }
                if (!root["robot_info"]["version"]["ultroso_ks106"].isNull()) {
                    diag_["robot_info"]["version"]["ultroso_ks106"] = root["robot_info"]["version"]["ultroso_ks106"];
                }
                if (!root["robot_info"]["version"]["ultroso_ks136"].isNull()) {
                    diag_["robot_info"]["version"]["ultroso_ks136"] = root["robot_info"]["version"]["ultroso_ks136"];
                }
            }

            if (!root["robot_info"]["battery"].isNull()) {
                // when we receive battery info , set communication to true
                bms_communication_ok_ = 1;
                struct timeval bms_recv_this;
                gettimeofday(&bms_recv_this,NULL);
                bms_info_recv_last_ = bms_recv_this.tv_sec;

                if (!root["robot_info"]["battery"]["voltage"].isNull()) {
                    diag_["robot_info"]["battery"]["voltage"] = root["robot_info"]["battery"]["voltage"];
                }
                if (!root["robot_info"]["battery"]["current"].isNull()) {
                    diag_["robot_info"]["battery"]["current"] = root["robot_info"]["battery"]["current"];
                }
                if (!root["robot_info"]["battery"]["level"].isNull()) {
                    diag_["robot_info"]["battery"]["level"] = root["robot_info"]["battery"]["level"];
                    int battery_level;
                    battery_level = diag_["robot_info"]["battery"]["level"].asInt();
                    checkBatteryLevel(battery_level);
                }
                if (!root["robot_info"]["battery"]["temp_max"].isNull()) {
                    diag_["robot_info"]["battery"]["temp_max"] = root["robot_info"]["battery"]["temp_max"];
                }
                if (!root["robot_info"]["battery"]["temp_min"].isNull()) {
                    diag_["robot_info"]["battery"]["temp_min"] = root["robot_info"]["battery"]["temp_min"];
                    //checkBatteryTempLowEvent();
                }
                if (!root["robot_info"]["battery"]["status"].isNull()) {
                    // receive charging status
                    diag_["robot_info"]["battery"]["status"] = root["robot_info"]["battery"]["status"];
                    
                    int charging_status;
                    charging_status = diag_["robot_info"]["battery"]["status"].asInt();
                    //log_info("%s charging status: %d",__FUNCTION__, charging_status);
                    if(charging_status == 1) // battery is charging
                    {
                        isBmsCharging_ = true;
                    }
                    else
                    {
                        isBmsCharging_ = false;
                    }

                    #if 0
                    
                    // charging finished , if the ptz is power up again, we need to delay the diag after 3 minutes again
                    if(charging_status_last_ == 1 && charging_status != 1)
                    {
                        // need to restart the system diag timer
                        log_warn("%s charging finish , restart the system diag timer",__FUNCTION__); // system may be restart again without cut off the power of x86
                        system_delay_check_ = false;

                        if(pSelfDiagTimer != NULL)
                        {
                            //pSelfDiagTimer->DeleteThread();
                            delete pSelfDiagTimer;
                            pSelfDiagTimer = new CTimer("self_check_diag_timer");
                            pSelfDiagTimer->AsyncOnce(3* 60 * 1000, &Diagnostics::sysDiagDelayProc, this);
                        }
                    }


                    charging_status_last_ = charging_status;
                    #endif
                }
                if (!root["robot_info"]["battery"]["charge_cnt"].isNull()) {
                    diag_["robot_info"]["battery"]["charge_cnt"] = root["robot_info"]["battery"]["charge_cnt"];
                }
                if (!root["robot_info"]["battery"]["discharge_cnt"].isNull()) {
                    diag_["robot_info"]["battery"]["discharge_cnt"] = root["robot_info"]["battery"]["discharge_cnt"];
                }
                if (!root["robot_info"]["battery"]["health"].isNull()) {
                    diag_["robot_info"]["battery"]["health"] = root["robot_info"]["battery"]["health"];
                }
                if (!root["robot_info"]["battery"]["bat_num"].isNull()) {
                    diag_["robot_info"]["battery"]["bat_num"] = root["robot_info"]["battery"]["bat_num"];
                }
                if (!root["robot_info"]["battery"]["relay_status"].isNull()) {
                    diag_["robot_info"]["battery"]["relay_status"] = root["robot_info"]["battery"]["relay_status"];
                }
                if (!root["robot_info"]["battery"]["charge_status"].isNull()) {
                    diag_["robot_info"]["battery"]["charge_status"] = root["robot_info"]["battery"]["charge_status"];
                }
                if (!root["robot_info"]["battery"]["discharge_status"].isNull()) {
                    diag_["robot_info"]["battery"]["discharge_status"] = root["robot_info"]["battery"]["discharge_status"];
                }
                if (!root["robot_info"]["battery"]["vstatus"].isNull()) {
                    diag_["robot_info"]["battery"]["vstatus"] = root["robot_info"]["battery"]["vstatus"];
                }
                if (!root["robot_info"]["battery"]["cstatus"].isNull()) {
                    diag_["robot_info"]["battery"]["cstatus"] = root["robot_info"]["battery"]["cstatus"];
                }
                if (!root["robot_info"]["battery"]["tstatus"].isNull()) {
                    diag_["robot_info"]["battery"]["tstatus"] = root["robot_info"]["battery"]["tstatus"];
                }
                if (!root["robot_info"]["battery"]["alarm"].isNull()) {
                    diag_["robot_info"]["battery"]["alarm"] = root["robot_info"]["battery"]["alarm"];
                }
                if (!root["robot_info"]["battery"]["nominal_voltage"].isNull()) {
                    diag_["robot_info"]["battery"]["nominal_voltage"] = root["robot_info"]["battery"]["nominal_voltage"];
                }
                if (!root["robot_info"]["battery"]["nominal_current"].isNull()) {
                    diag_["robot_info"]["battery"]["nominal_current"] = root["robot_info"]["battery"]["nominal_current"];
                }
                if (!root["robot_info"]["battery"]["voltage_max"].isNull()) {
                    diag_["robot_info"]["battery"]["voltage_max"] = root["robot_info"]["battery"]["voltage_max"];
                }
                if (!root["robot_info"]["battery"]["voltage_min"].isNull()) {
                    diag_["robot_info"]["battery"]["voltage_min"] = root["robot_info"]["battery"]["voltage_min"];
                }
                if (!root["robot_info"]["battery"]["voltage_max_num"].isNull()) {
                    diag_["robot_info"]["battery"]["voltage_max_num"] = root["robot_info"]["battery"]["voltage_max_num"];
                }
                if (!root["robot_info"]["battery"]["voltage_min_num"].isNull()) {
                    diag_["robot_info"]["battery"]["voltage_min_num"] = root["robot_info"]["battery"]["voltage_min_num"];
                }                
                if (!root["robot_info"]["battery"]["voltage_max_serial_num"].isNull()) {
                    diag_["robot_info"]["battery"]["voltage_max_serial_num"] = root["robot_info"]["battery"]["voltage_max_serial_num"];
                }
                if (!root["robot_info"]["battery"]["voltage_min_serial_num"].isNull()) {
                    diag_["robot_info"]["battery"]["voltage_min_serial_num"] = root["robot_info"]["battery"]["voltage_min_serial_num"];
                }                
                if (!root["robot_info"]["battery"]["temp_max_pack_num"].isNull()) {
                    diag_["robot_info"]["battery"]["temp_max_pack_num"] = root["robot_info"]["battery"]["temp_max_pack_num"];
                }
                if (!root["robot_info"]["battery"]["temp_min_pack_num"].isNull()) {
                    diag_["robot_info"]["battery"]["temp_min_pack_num"] = root["robot_info"]["battery"]["temp_min_pack_num"];
                }
                if (!root["robot_info"]["battery"]["temp_max_pos"].isNull()) {
                    diag_["robot_info"]["battery"]["temp_max_pos"] = root["robot_info"]["battery"]["temp_max_pos"];
                }
                if (!root["robot_info"]["battery"]["temp_min_pos"].isNull()) {
                    diag_["robot_info"]["battery"]["temp_min_pos"] = root["robot_info"]["battery"]["temp_min_pos"];
                }
                if (!root["robot_info"]["battery"]["life_percent"].isNull()) {
                    diag_["robot_info"]["battery"]["life_percent"] = root["robot_info"]["battery"]["life_percent"];
                }
                if (!root["robot_info"]["battery"]["charge_mos_status"].isNull()) {
                    diag_["robot_info"]["battery"]["charge_mos_status"] = root["robot_info"]["battery"]["charge_mos_status"];
                }
                if (!root["robot_info"]["battery"]["discharge_mos_status"].isNull()) {
                    diag_["robot_info"]["battery"]["discharge_mos_status"] = root["robot_info"]["battery"]["discharge_mos_status"];
                }
                if (!root["robot_info"]["battery"]["external_io_status"].isNull()) {
                    diag_["robot_info"]["battery"]["external_io_status"] = root["robot_info"]["battery"]["external_io_status"];
                }
                if (!root["robot_info"]["battery"]["charge_discharge_cycles"].isNull()) {
                    diag_["robot_info"]["battery"]["charge_discharge_cycles"] = root["robot_info"]["battery"]["charge_discharge_cycles"];
                }
                if (!root["robot_info"]["battery"]["shutdown_indicator"].isNull()) {
                    diag_["robot_info"]["battery"]["shutdown_indicator"] = root["robot_info"]["battery"]["shutdown_indicator"];
                }
                if (!root["robot_info"]["battery"]["warning_status"].isNull()) {
                    diag_["robot_info"]["battery"]["warning_status"] = root["robot_info"]["battery"]["warning_status"];
                }
                if (!root["robot_info"]["battery"]["warning_low"].isNull()) {
                    diag_["robot_info"]["battery"]["warning_low"] = root["robot_info"]["battery"]["warning_low"];
                }
                if (!root["robot_info"]["battery"]["warning_mid"].isNull()) {
                    diag_["robot_info"]["battery"]["warning_mid"] = root["robot_info"]["battery"]["warning_mid"];
                }
                if (!root["robot_info"]["battery"]["warning_high"].isNull()) {
                    diag_["robot_info"]["battery"]["warning_high"] = root["robot_info"]["battery"]["warning_high"];
                }

                if(!root["robot_info"]["battery"]["warning_high"].isNull() && !root["robot_info"]["battery"]["warning_mid"].isNull() && !root["robot_info"]["battery"]["warning_low"].isNull())
                {
                    int low_byte;
                    int mid_byte;
                    int high_byte;
                    int bat_error;
                    low_byte = root["robot_info"]["battery"]["warning_low"].asInt();
                    mid_byte = root["robot_info"]["battery"]["warning_mid"].asInt();
                    high_byte = root["robot_info"]["battery"]["warning_high"].asInt();

                    bat_error = deterBatteryError((unsigned short)low_byte, (unsigned short)mid_byte, (unsigned short)high_byte);
                    if(bat_error < 0)
                    {
                        diag_["robot_info"]["battery"]["error"] = 1;
                        bms_warning_stats_ = 1;
                    }
                    else
                    {
                        diag_["robot_info"]["battery"]["error"] = 0;
                        bms_warning_stats_ = 0;
                    }

                    checkBatteryWarningEvent(low_byte, mid_byte, high_byte);
                    //checkBMSWhenStartUp(bat_error);
                    //checkBMSErrorEvent(bat_error);
                }

            }

            if (!root["robot_info"]["4g"].isNull()) {
                if (!root["robot_info"]["4g"]["error"].isNull()) {
                    diag_["robot_info"]["4g"]["error"] = root["robot_info"]["4g"]["error"];
                }
                if (!root["robot_info"]["4g"]["level"].isNull()) {
                    diag_["robot_info"]["4g"]["level"] = root["robot_info"]["4g"]["level"];
                }
            }

            if (!root["robot_info"]["doublecom"].isNull()) {
                if (!root["robot_info"]["doublecom"]["level"].isNull()) {
                    diag_["robot_info"]["doublecom"]["level"] = root["robot_info"]["doublecom"]["level"];
                }

                if (!root["robot_info"]["doublecom"]["connect_status"].isNull()) {
                    diag_["robot_info"]["doublecom"]["connect_status"] = root["robot_info"]["doublecom"]["connect_status"];
                }

                if (!root["robot_info"]["doublecom"]["login_status"].isNull()) {
                    diag_["robot_info"]["doublecom"]["login_status"] = root["robot_info"]["doublecom"]["login_status"];
                }

                if (!root["robot_info"]["doublecom"]["sig_val"].isNull()) {
                    diag_["robot_info"]["doublecom"]["sig_val"] = root["robot_info"]["doublecom"]["sig_val"];
                }

            }

            if (!root["robot_info"]["ads"].isNull()) {
                if (!root["robot_info"]["ads"]["status"].isNull()) {
                    diag_["robot_info"]["ads"]["status"] = root["robot_info"]["ads"]["status"];
                }

                if (!root["robot_info"]["ads"]["error"].isNull()) {
                    diag_["robot_info"]["ads"]["error"] = root["robot_info"]["ads"]["error"];
                }
            }

            if (!root["robot_info"]["sim"].isNull()) {
                if (!root["robot_info"]["sim"]["iccid"].isNull()) {
                    diag_ ["robot_info"]["sim"]["iccid"] = root["robot_info"]["sim"]["iccid"];
                }
                if (!root["robot_info"]["sim"]["imsi"].isNull()) {
                    diag_ ["robot_info"]["sim"]["imsi"] = root["robot_info"]["sim"]["imsi"];
                }
                if (!root["robot_info"]["sim"]["cellid"].isNull()) {
                    diag_ ["robot_info"]["sim"]["cellid"] = root["robot_info"]["sim"]["cellid"];
                }
            }

            if (!root["robot_info"]["router"].isNull()) {
                if (!root["robot_info"]["router"]["imei"].isNull()) {
                    diag_["robot_info"]["router"]["imei"] = root["robot_info"]["router"]["imei"];
                }
                if (!root["robot_info"]["router"]["mac"].isNull()) {
                    diag_ ["robot_info"]["router"]["mac"] = root["robot_info"]["router"]["mac"];
                }
                if(!root["robot_info"]["router"]["wan"]["ip"].isNull()){
                   diag_ ["robot_info"]["router"]["wan"]["ip"] = root["robot_info"]["router"]["wan"]["ip"];
                }
            }

            if (!root["robot_info"]["gps"].isNull()) {
                updateGpsRecvTime();
                if (!root["robot_info"]["gps"]["error"].isNull()) {
                    diag_["robot_info"]["gps"]["error"] = root["robot_info"]["gps"]["error"];
                }
                if (!root["robot_info"]["gps"]["lati"].isNull()) {
                    diag_["robot_info"]["gps"]["lati"] = root["robot_info"]["gps"]["lati"];
                }
                if (!root["robot_info"]["gps"]["long"].isNull()) {
                    diag_["robot_info"]["gps"]["long"] = root["robot_info"]["gps"]["long"];
                }
                if (!root["robot_info"]["gps"]["alti"].isNull()) {
                    diag_["robot_info"]["gps"]["alti"] = root["robot_info"]["gps"]["alti"];
                }

            }

            if (!root["robot_info"]["main_lidar"].isNull()) {
                if (!root["robot_info"]["main_lidar"]["error"].isNull()) {
                    diag_["robot_info"]["main_lidar"]["error"] = root["robot_info"]["main_lidar"]["error"];
                }
                if (!root["robot_info"]["main_lidar"]["data"].isNull()) {
                    diag_["robot_info"]["main_lidar"]["data"] = root["robot_info"]["main_lidar"]["data"];
                }
            }

            if (!root["robot_info"]["slave_lidar"].isNull()) {
                if (!root["robot_info"]["slave_lidar"]["error"].isNull()) {
                    diag_["robot_info"]["slave_lidar"]["error"] = root["robot_info"]["slave_lidar"]["error"];
                }
                if (!root["robot_info"]["slave_lidar"]["data"].isNull()) {
                    diag_["robot_info"]["slave_lidar"]["data"] = root["robot_info"]["slave_lidar"]["data"];
                }
            }

            if (!root["robot_info"]["ultrasound"].isNull()) {
                if (!root["robot_info"]["ultrasound"]["error"].isNull()) {
                    diag_["robot_info"]["ultrasound"]["error"] = root["robot_info"]["ultrasound"]["error"];
                }
                if (!root["robot_info"]["ultrasound"]["data"].isNull()) {
                    diag_["robot_info"]["ultrasound"]["data"] = root["robot_info"]["ultrasound"]["data"];

                     checkUltraStartup();
                }

                if (diag_["robot_info"]["ultrasound"]["error"].asInt() != 0)
                {
                    checkUltraErrorEvent();
                }
            }

            if (!root["robot_info"]["chassis_driver"].isNull()) {
                if (!root["robot_info"]["chassis_driver"]["error"].isNull()) {
                    diag_["robot_info"]["chassis_driver"]["error"] = root["robot_info"]["chassis_driver"]["error"];
                }
                if (!root["robot_info"]["chassis_driver"]["error1"].isNull()) {
                    diag_["robot_info"]["chassis_driver"]["error1"] = root["robot_info"]["chassis_driver"]["error1"];
                }
                if (!root["robot_info"]["chassis_driver"]["temp_ic"].isNull()) {
                    diag_["robot_info"]["chassis_driver"]["temp_ic"] = root["robot_info"]["chassis_driver"]["temp_ic"];
                }
                if (!root["robot_info"]["chassis_driver"]["temp_motor_left"].isNull()) {
                    diag_["robot_info"]["chassis_driver"]["temp_motor_left"] = root["robot_info"]["chassis_driver"]["temp_motor_left"];
                }
                if (!root["robot_info"]["chassis_driver"]["temp_motor_right"].isNull()) {
                    diag_["robot_info"]["chassis_driver"]["temp_motor_right"] = root["robot_info"]["chassis_driver"]["temp_motor_right"];
                }
                if (!root["robot_info"]["chassis_driver"]["bat_voltage"].isNull()) {
                    diag_["robot_info"]["chassis_driver"]["bat_voltage"] = root["robot_info"]["chassis_driver"]["bat_voltage"];
                }
                if (!root["robot_info"]["chassis_driver"]["motor_current"].isNull()) {
                    diag_["robot_info"]["chassis_driver"]["motor_current"] = root["robot_info"]["chassis_driver"]["motor_current"];
                }
                if (!root["robot_info"]["chassis_driver"]["bat_current"].isNull()) {
                    diag_["robot_info"]["chassis_driver"]["bat_current"] = root["robot_info"]["chassis_driver"]["bat_current"];
                }
                if (!root["robot_info"]["chassis_driver"]["current_control_priority"].isNull()) {
                    diag_["robot_info"]["chassis_driver"]["current_control_priority"] = root["robot_info"]["chassis_driver"]["current_control_priority"];
                }
            }

            if (!root["robot_info"]["wifi"].isNull()) {
                if (!root["robot_info"]["wifi"]["error"].isNull()) {
                    diag_["robot_info"]["wifi"]["error"] = root["robot_info"]["wifi"]["error"];
                }
                if (!root["robot_info"]["wifi"]["level"].isNull()) {
                    diag_["robot_info"]["wifi"]["level"] = root["robot_info"]["wifi"]["level"];
                }
                if (!root["robot_info"]["wifi"]["ssid"].isNull()) {
                    diag_["robot_info"]["wifi"]["ssid"] = root["robot_info"]["wifi"]["ssid"];
                }
            }

            if (!root["robot_info"]["power"].isNull()) {
                if (!root["robot_info"]["power"]["disperse"].isNull()) {
                    diag_["robot_info"]["power"]["disperse"] = root["robot_info"]["power"]["disperse"];
                }
                if (!root["robot_info"]["power"]["netswitch"].isNull()) {
                    diag_["robot_info"]["power"]["netswitch"] = root["robot_info"]["power"]["netswitch"];
                }
                if (!root["robot_info"]["power"]["gps"].isNull()) {
                    diag_["robot_info"]["power"]["gps"] = root["robot_info"]["power"]["gps"];
                }
                if (!root["robot_info"]["power"]["gs"].isNull()) {
                    diag_["robot_info"]["power"]["gs"] = root["robot_info"]["power"]["gs"];
                }
                if (!root["robot_info"]["power"]["slave_lidar"].isNull()) {
                    diag_["robot_info"]["power"]["slave_lidar"] = root["robot_info"]["power"]["slave_lidar"];
                }
                if (!root["robot_info"]["power"]["main_lidar"].isNull()) {
                    diag_["robot_info"]["power"]["main_lidar"] = root["robot_info"]["power"]["main_lidar"];
                }
                if (!root["robot_info"]["power"]["yuntai"].isNull()) {
                    diag_["robot_info"]["power"]["yuntai"] = root["robot_info"]["power"]["yuntai"];
                }
                if (!root["robot_info"]["power"]["m_fan_in"].isNull()) {
                    diag_["robot_info"]["power"]["m_fan_in"] = root["robot_info"]["power"]["m_fan_in"];
                }
                if (!root["robot_info"]["power"]["m_fan_out"].isNull()) {
                    diag_["robot_info"]["power"]["m_fan_out"] = root["robot_info"]["power"]["m_fan_out"];
                }
                if (!root["robot_info"]["power"]["ks136"].isNull()) {
                    diag_["robot_info"]["power"]["ks136"] = root["robot_info"]["power"]["ks136"];
                }
                if (!root["robot_info"]["power"]["ks106"].isNull()) {
                    diag_["robot_info"]["power"]["ks106"] = root["robot_info"]["power"]["ks106"];
                }
                if (!root["robot_info"]["power"]["imx"].isNull()) {
                    diag_["robot_info"]["power"]["imx"] = root["robot_info"]["power"]["imx"];
                }
            }

            if (!root["robot_info"]["gyro"].isNull()) {
                if (!root["robot_info"]["gyro"]["error"].isNull()) {
                    diag_["robot_info"]["gyro"]["error"] = root["robot_info"]["gyro"]["error"];
                }
            }

            if (!root["robot_info"]["odom"].isNull()) {
                if (!root["robot_info"]["odom"]["error"].isNull()) {
                    diag_["robot_info"]["odom"]["error"] = root["robot_info"]["odom"]["error"];
                }
                if (!root["robot_info"]["odom"]["odo"].isNull()) {
                    diag_["robot_info"]["odom"]["odo"] = root["robot_info"]["odom"]["odo"];
                }
                if (!root["robot_info"]["odom"]["speed_linear"].isNull()) {
                    diag_["robot_info"]["odom"]["speed_linear"] = root["robot_info"]["odom"]["speed_linear"];
                }
                if (!root["robot_info"]["odom"]["speed_theta"].isNull()) {
                    diag_["robot_info"]["odom"]["speed_theta"] = root["robot_info"]["odom"]["speed_theta"];
                }
            }

            if (!root["robot_info"]["sensor_liquid"].isNull()) {
                if (!root["robot_info"]["sensor_liquid"]["status"].isNull()) {
                    diag_["robot_info"]["sensor_liquid"]["status"] = root["robot_info"]["sensor_liquid"]["status"];
                }
            }

            if (!root["robot_info"]["sensor_hall"].isNull()) {
                if (!root["robot_info"]["sensor_hall"]["hall1"].isNull()) {
                    diag_["robot_info"]["sensor_hall"]["hall1"] = root["robot_info"]["sensor_hall"]["hall1"];
                }
                if (!root["robot_info"]["sensor_hall"]["hall2"].isNull()) {
                    diag_["robot_info"]["sensor_hall"]["hall2"] = root["robot_info"]["sensor_hall"]["hall2"];
                }
                if (!root["robot_info"]["sensor_hall"]["hall3"].isNull()) {
                    diag_["robot_info"]["sensor_hall"]["hall3"] = root["robot_info"]["sensor_hall"]["hall3"];
                }
            }

            if (!root["robot_info"]["temp_humi"].isNull()) {
                if (!root["robot_info"]["temp_humi"]["motor_left"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["motor_left"] = root["robot_info"]["temp_humi"]["motor_left"];
                }
                if (!root["robot_info"]["temp_humi"]["motor_right"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["motor_right"] = root["robot_info"]["temp_humi"]["motor_right"];
                }
                if (!root["robot_info"]["temp_humi"]["imx"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["imx"] = root["robot_info"]["temp_humi"]["imx"];
                }
                if (!root["robot_info"]["temp_humi"]["kuangshi"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["kuangshi"] = root["robot_info"]["temp_humi"]["kuangshi"];
                }
                if (!root["robot_info"]["temp_humi"]["gaussian"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["gaussian"] = root["robot_info"]["temp_humi"]["gaussian"];
                }
                if (!root["robot_info"]["temp_humi"]["temp_env"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["temp_env"] = root["robot_info"]["temp_humi"]["temp_env"];
                }
                if (!root["robot_info"]["temp_humi"]["humi_env"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["humi_env"] = root["robot_info"]["temp_humi"]["humi_env"];
                }

                if (!root["robot_info"]["temp_humi"]["sys_temp"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["sys_temp"] = root["robot_info"]["temp_humi"]["sys_temp"];
                    float fTemp_env;
                    fTemp_env = diag_["robot_info"]["temp_humi"]["sys_temp"].asFloat();
                    diag_["robot_info"]["temp_humi"]["temp_env"] = doubleTrim(fTemp_env); // use system monitor temp as temp env
                }
                if (!root["robot_info"]["temp_humi"]["sys_humi"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["sys_humi"] = root["robot_info"]["temp_humi"]["sys_humi"];
                }
                if (!root["robot_info"]["temp_humi"]["sys_NTC1"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["sys_NTC1"] = root["robot_info"]["temp_humi"]["sys_NTC1"];
                }
                if (!root["robot_info"]["temp_humi"]["sys_NTC2"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["sys_NTC2"] = root["robot_info"]["temp_humi"]["sys_NTC2"];
                }
                if (!root["robot_info"]["temp_humi"]["sys_NTC3"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["sys_NTC3"] = root["robot_info"]["temp_humi"]["sys_NTC3"];
                }
                if (!root["robot_info"]["temp_humi"]["sys_NTC4"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["sys_NTC4"] = root["robot_info"]["temp_humi"]["sys_NTC4"];
                }

                if (!root["robot_info"]["temp_humi"]["chassis_temp"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["chassis_temp"] = root["robot_info"]["temp_humi"]["chassis_temp"];
                }
                if (!root["robot_info"]["temp_humi"]["chassis_humi"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["chassis_humi"] = root["robot_info"]["temp_humi"]["chassis_humi"];
                }
                if (!root["robot_info"]["temp_humi"]["chassis_NTC1"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["chassis_NTC1"] = root["robot_info"]["temp_humi"]["chassis_NTC1"];
                }
                if (!root["robot_info"]["temp_humi"]["chassis_NTC2"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["chassis_NTC2"] = root["robot_info"]["temp_humi"]["chassis_NTC2"];
                }
                if (!root["robot_info"]["temp_humi"]["chassis_NTC3"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["chassis_NTC3"] = root["robot_info"]["temp_humi"]["chassis_NTC3"];
                }
                if (!root["robot_info"]["temp_humi"]["chassis_NTC4"].isNull()) {
                    diag_["robot_info"]["temp_humi"]["chassis_NTC4"] = root["robot_info"]["temp_humi"]["chassis_NTC4"];
                }
            }

            if (!root["robot_info"]["fan"].isNull()) {
                if (!root["robot_info"]["fan"]["bottom"].isNull()) {
                    if (!root["robot_info"]["fan"]["bottom"]["error"].isNull()) {
                        diag_["robot_info"]["fan"]["bottom"]["error"] = root["robot_info"]["fan"]["bottom"]["error"];
                    }
                    if (!root["robot_info"]["fan"]["bottom"]["speed_in"].isNull()) {
                        diag_["robot_info"]["fan"]["bottom"]["speed_in"] = root["robot_info"]["fan"]["bottom"]["speed_in"];
                    }
                    if (!root["robot_info"]["fan"]["bottom"]["speed_out"].isNull()) {
                        diag_["robot_info"]["fan"]["bottom"]["speed_out"] = root["robot_info"]["fan"]["bottom"]["speed_out"];
                    }
                }
                if (!root["robot_info"]["fan"]["middle"].isNull()) {
                    if (!root["robot_info"]["fan"]["middle"]["error"].isNull()) {
                        diag_["robot_info"]["fan"]["middle"]["error"] = root["robot_info"]["fan"]["middle"]["error"];
                    }
                    if (!root["robot_info"]["fan"]["middle"]["speed_in"].isNull()) {
                        diag_["robot_info"]["fan"]["middle"]["speed_in"] = root["robot_info"]["fan"]["middle"]["speed_in"];
                    }
                    if (!root["robot_info"]["fan"]["middle"]["speed_out"].isNull()) {
                        diag_["robot_info"]["fan"]["middle"]["speed_out"] = root["robot_info"]["fan"]["middle"]["speed_out"];
                    }
                }

            }

            if (!root["robot_info"]["brake"].isNull()) {
                if (!root["robot_info"]["brake"]["button"].isNull()) {
                    diag_["robot_info"]["brake"]["button"] = root["robot_info"]["brake"]["button"];
                }
                if (!root["robot_info"]["brake"]["front_bumper_type"].isNull()) {
                    diag_["robot_info"]["brake"]["front_bumper_type"] = root["robot_info"]["brake"]["front_bumper_type"];
                }
                if (!root["robot_info"]["brake"]["tear_bumper_type"].isNull()) {
                    diag_["robot_info"]["brake"]["tear_bumper_type"] = root["robot_info"]["brake"]["tear_bumper_type"];
                }
                if (!root["robot_info"]["brake"]["charge"].isNull()) {
                    diag_["robot_info"]["brake"]["charge"] = root["robot_info"]["brake"]["charge"];
                }
                if (!root["robot_info"]["brake"]["charge_bumper"].isNull()) {
                    diag_["robot_info"]["brake"]["charge_bumper"] = root["robot_info"]["brake"]["charge_bumper"];
                }
                if (!root["robot_info"]["brake"]["front_bumper"].isNull()) {
                    diag_["robot_info"]["brake"]["front_bumper"] = root["robot_info"]["brake"]["front_bumper"];
                    int bumper_status = root["robot_info"]["brake"]["front_bumper"].asInt();
                    log_info("%s, bumper_status:%d", __FUNCTION__, bumper_status);
                    checkCollisionEvent(bumper_status);
                }
                if (!root["robot_info"]["brake"]["tear_bumper"].isNull()) {
                    diag_["robot_info"]["brake"]["tear_bumper"] = root["robot_info"]["brake"]["tear_bumper"];
                }

                //checkCollisionTrigEvent();

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                if (!root["robot_info"]["brake"]["source"].isNull()) 
                {
                    if (!root["robot_info"]["brake"]["source"]["button"].isNull()) {
                        diag_["robot_info"]["brake"]["source"]["button"] = root["robot_info"]["brake"]["source"]["button"];
                        int button_status;
                        button_status = diag_["robot_info"]["brake"]["source"]["button"].asInt();
                        checkEmergeButtonWhenStartUp(button_status);
                        checkEmergeButtonEvent(button_status);

                    }

                    if (!root["robot_info"]["brake"]["source"]["charge"].isNull()) {
                        diag_["robot_info"]["brake"]["source"]["charge"] = root["robot_info"]["brake"]["source"]["charge"];
                    }
                    if (!root["robot_info"]["brake"]["source"]["charge_bumper"].isNull()) {
                        diag_["robot_info"]["brake"]["source"]["charge_bumper"] = root["robot_info"]["brake"]["source"]["charge_bumper"];
                    }
                    if (!root["robot_info"]["brake"]["source"]["front_bumper"].isNull()) {
                        diag_["robot_info"]["brake"]["source"]["front_bumper"] = root["robot_info"]["brake"]["source"]["front_bumper"];
                    }
                    if (!root["robot_info"]["brake"]["source"]["tear_bumper"].isNull()) {
                        diag_["robot_info"]["brake"]["source"]["tear_bumper"] = root["robot_info"]["brake"]["source"]["tear_bumper"];
                    }


                    if (!root["robot_info"]["brake"]["source"]["can"].isNull()) {
                        diag_["robot_info"]["brake"]["source"]["can"] = root["robot_info"]["brake"]["source"]["can"];
                    }
                    if (!root["robot_info"]["brake"]["source"]["iap"].isNull()) {
                        diag_["robot_info"]["brake"]["source"]["iap"] = root["robot_info"]["brake"]["source"]["iap"];
                    }
                    if (!root["robot_info"]["brake"]["source"]["reboot"].isNull()) {
                        diag_["robot_info"]["brake"]["source"]["reboot"] = root["robot_info"]["brake"]["source"]["reboot"];
                    }

                }

                if (!root["robot_info"]["brake"]["control"].isNull()) {
                    if (!root["robot_info"]["brake"]["control"]["CMotor_brake"].isNull()) {
                        diag_["robot_info"]["brake"]["control"]["CMotor_brake"] = root["robot_info"]["brake"]["control"]["CMotor_brake"];
                    }
                    if (!root["robot_info"]["brake"]["control"]["CEmergency_brake"].isNull()) {
                        diag_["robot_info"]["brake"]["control"]["CEmergency_brake"] = root["robot_info"]["brake"]["control"]["CEmergency_brake"];
                    }
                    if (!root["robot_info"]["brake"]["control"]["W_software_brake"].isNull()) {
                        diag_["robot_info"]["brake"]["control"]["W_software_brake"] = root["robot_info"]["brake"]["control"]["W_software_brake"];
                    }
                    if (!root["robot_info"]["brake"]["control"]["button_hardware_brake"].isNull()) {
                        diag_["robot_info"]["brake"]["control"]["button_hardware_brake"] = root["robot_info"]["brake"]["control"]["button_hardware_brake"];
                    }
                }

                // robot_braked
                bool robot_braked = checkRobotBrake();
                shm::RobotBraked shm_robot_braked;
                shm_robot_braked.braked = robot_braked;
                shm::iMemory_write_RobotBraked(&shm_robot_braked);

                atris_msgs::RobotBraked robot_braked_msg;
                robot_braked_msg.braked = robot_braked;
                robot_braked_pub_.publish(robot_braked_msg);

                // reboot_braked
                bool reboot_braked = checkRebootBrake();
                shm::RebootBraked shm_reboot_braked;
                shm_reboot_braked.braked = reboot_braked;
                shm::iMemory_write_RebootBraked(&shm_reboot_braked);

                atris_msgs::RebootBraked reboot_braked_msg;
                reboot_braked_msg.braked = reboot_braked;
                reboot_braked_pub_.publish(reboot_braked_msg);
            }

            if (!root["robot_info"]["charge"].isNull()) {
                if (!root["robot_info"]["charge"]["switch"].isNull()) {
                    diag_["robot_info"]["charge"]["switch"] = root["robot_info"]["charge"]["switch"];
                }
                if (!root["robot_info"]["charge"]["electrodes_status"].isNull()) {
                    diag_["robot_info"]["charge"]["electrodes_status"] = root["robot_info"]["charge"]["electrodes_status"];
                }
                if (!root["robot_info"]["charge"]["electrodes_voltage"].isNull()) {
                    diag_["robot_info"]["charge"]["electrodes_voltage"] = root["robot_info"]["charge"]["electrodes_voltage"];
                }
                if (!root["robot_info"]["charge"]["bluetooth"].isNull()) {
                    diag_["robot_info"]["charge"]["bluetooth"] = root["robot_info"]["charge"]["bluetooth"];
                }
            }
            if (!root["robot_info"]["gaussian_status"].isNull()) {
                diag_["robot_info"]["gaussian_status"] = root["robot_info"]["gaussian_status"];
            }
        }
    }
}

void Diagnostics::on_recv_upgrade_stats(const atris_msgs::UpgradeStatus& msg)
{
    if(isUpgrading != msg.upgrading)
    {
        notifyUpgradeEvent(msg.upgrading);
    }

    isUpgrading = msg.upgrading;
}

// notify change of control variable to shadow server
void Diagnostics::notifyRobotInfoToShadow(void)
{
    log_info("%s",__FUNCTION__);
    Json::FastWriter jwriter;
    atris_msgs::SignalMessage diag_msg;
    Json::Value root;
    shm::Robot shmrbt;
    shm::CompanyId shmcomid;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    Json::Value tmp = getValue();

    shm::iMemory_read_Robot(&shmrbt);
    shm::iMemory_read_CompanyId(&shmcomid);

    log_info("%s sn = %s",__FUNCTION__, shmrbt.robot.sn);

    root["title"] = "notify_robot_info";
    root["content"] = tmp;

    std::string company_id = shmcomid.company_id;
    root["content"]["robot"]["sn"] = shmrbt.robot.sn;
    root["content"]["robot"]["power"] = "on";
    root["content"]["robot"]["company"] = company_id.empty() ? "UBT_ATRIS" : company_id;
    root["content"]["gps_info"]["state"] = -1;
    root["content"]["gps_info"]["lati"] = "0";
    root["content"]["gps_info"]["long"] = "0";

    
    if (!tmp["robot_info"].isNull() && !tmp["robot_info"]["gps"].isNull()) {
        if (!tmp["robot_info"]["gps"]["error"].isNull()) {
            root["content"]["gps_info"]["state"] = tmp["robot_info"]["gps"]["error"].asInt();
        }
        if (!tmp["robot_info"]["gps"]["lati"].isNull()) {
            root["content"]["gps_info"]["lati"] = tmp["robot_info"]["gps"]["lati"].asString();
        }
        if (!tmp["robot_info"]["gps"]["long"].isNull()) {
            root["content"]["gps_info"]["long"] = tmp["robot_info"]["gps"]["long"].asString();
        }
        
        if (!tmp["robot_info"]["gps"]["alti"].isNull()) {
            root["content"]["gps_info"]["alti"] = tmp["robot_info"]["gps"]["alti"];
        }
    }
    
    if (!tmp["robot_info"]["power"].isNull() && !tmp["robot_info"]["power"]["imx"].isNull()) {
        root["content"]["robot"]["power"] = (tmp["robot_info"]["power"]["imx"].asInt() != 0) ? "on" : "off";
    }

    Json::Value remote_mqtt_info;

    remote_mqtt_info["domain"] = "setting";
    remote_mqtt_info["productName"] = "Atris";
    remote_mqtt_info["sn"] = shmrbt.robot.sn;
    remote_mqtt_info["state"]["reported"] = root["content"];
    remote_mqtt_info["type"] = "update";
    remote_mqtt_info["title"] = "notify_robot_info";

    diag_msg.type = "remote_mqtt";
    diag_msg.account = shmrbt.robot.receiver;
    diag_msg.msgID = uid.str();
    diag_msg.title = "notify_robot_info";
    diag_msg.msg = jwriter.write(remote_mqtt_info);

    mqtt_signal_pub_.publish(diag_msg);

}


Json::Value Diagnostics::diagnosticsRobotInfo(std::string id, int64_t timestamp) {
    shm::Robot shmrbt;
    shm::CompanyId shmcomid;
    shm::iMemory_read_Robot(&shmrbt);
    shm::iMemory_read_CompanyId(&shmcomid);
    float robot_work_time;
    Json::Value response = getValue();
    getSysUpTime(robot_work_time);

    std::string company_id = shmcomid.company_id;
    response["robot_info"]["base"]["sn"] = shmrbt.robot.sn;
    response["robot_info"]["base"]["binded"] =  shmrbt.robot.accid;
    response["robot_info"]["base"]["company"] =  company_id.empty() ? "UBT_ATRIS" : company_id;
    response["robot_info"]["base"]["work_time"] = robot_work_time;
    response["robot_info"]["shaked"]["value"] = shmrbt.appdata.shaked;
    response["robot_info"]["speed"]["value"] = shmrbt.appdata.chassis_speed;
    response["robot_info"]["volume"]["muted"] = shmrbt.appdata.muted;
    response["robot_info"]["volume"]["value"] = shmrbt.appdata.volume;
    response["robot_info"]["light"]["w_status"] = 0;
    response["robot_info"]["light"]["rb_status"] = 0;
    response["robot_info"]["ppplay"]["status"] = 0;
    response["robot_info"]["ppplay"]["name"] = "";
    response["robot_info"]["ppplay"]["pts"] = 0;
    response["robot_info"]["ppplay"]["duration"] = 0;
    response["robot_info"]["ppplay"]["interval"] = shmrbt.appdata.play_interval*1000;
    response["robot_info"]["disperse"]["status"] = 0;
    response["robot_info"]["patrol"]["status"] = 0;
    response["robot_info"]["camera"]["status"] = 0;
    response["robot_info"]["camera"]["ptz_ip"] = Config::get_instance()->ptz_ip;
    response["robot_info"]["camera"]["nvr_ip"] = Config::get_instance()->nvr_ip;
    response["robot_info"]["voip"]["volume"] = shmrbt.appdata.sip_volume;
    response["robot_info"]["voip"]["number"] = shmrbt.appdata.sip_num;
    response["robot_info"]["voip"]["status"] = getVopCallingStatus();
    response["robot_info"]["tts"]["lng"] = shmrbt.appdata.tts_lng;
    response["robot_info"]["tts"]["speaker"] = shmrbt.appdata.tts_speaker;
    response["robot_info"]["tts"]["enable"] = shmrbt.appdata.tts_enable;

    response["robot_info"]["version"]["imx"] = "";
    response["robot_info"]["version"]["power"] = "";
    response["robot_info"]["version"]["gs"] = "";
    response["robot_info"]["version"]["battery_monitor"] = "";
    response["robot_info"]["version"]["bms"] = "";
    #if 0
    {
        atris_msgs::GetLampStatus status;
        if (get_lamp_status_srv_client_.call(status)) {
            response["robot_info"]["light"]["w_status"] = status.response.w_status;
            response["robot_info"]["light"]["rb_status"] = status.response.rb_status;
        }
    }
    #endif
    #if 0
    {
        atris_msgs::GetStandbyMode status;
        if (get_standby_mode_srv_client_.call(status)) {
            response["robot_info"]["base"]["machine_status"] = status.response.standby_mode;

        }
    }
    #endif

    {
        atris_msgs::GetPPPlayingStatus status;
        if (get_ppplaying_status_srv_client_.call(status)) {
            response["robot_info"]["ppplay"]["status"] = status.response.status;
            response["robot_info"]["ppplay"]["name"] = status.response.name;
            response["robot_info"]["ppplay"]["pts"] = status.response.pts;
            response["robot_info"]["ppplay"]["duration"] = status.response.duration;
        }
    }
    {
        atris_msgs::GetDisperseStatus status;
        if (get_disperse_status_srv_client_.call(status)) {
            response["robot_info"]["disperse"]["status"]  = status.response.dispersing ? 1 : 0;
        }
    }
    {
        atris_msgs::GetPatrolStatus status;
        if (get_patrol_status_srv_client_.call(status)) {
            response["robot_info"]["patrol"]["status"]  = status.response.status;
        }
    }
    {
        requestPtzParam();
        atris_msgs::GetPtzStatus status;
        if (get_ptz_status_srv_client_.call(status)) {
            response["robot_info"]["camera"]["status"]  = status.response.status;
            response["robot_info"]["camera"]["pan_angle"] = doubleTrim(status.response.pan_angle);
            response["robot_info"]["camera"]["tilt_angle"] = doubleTrim(status.response.tilt_angle);
            response["robot_info"]["camera"]["zoom_value"] = doubleTrim(status.response.zoom_value);
            response["robot_info"]["camera"]["light_status"] = status.response.light_status;
            response["robot_info"]["camera"]["wiper_status"] = status.response.wiper_status;
            int ptz_error_status;
            ptz_error_status = response["robot_info"]["camera"]["status"].asInt();
            //log_info("%s ptz login status : %d",__FUNCTION__, ptz_error_status);
            if(ptz_error_status == 0)
            {
                response["robot_info"]["camera"]["error"] = 1;
            }
            else
            {
                response["robot_info"]["camera"]["error"] = 0;
            }

        }
    }
    {
        atris_msgs::GetSwVersion status;
        if (get_sw_version_srv_client_.call(status)) {
            response["robot_info"]["version"]["imx"]  = status.response.imx_version;
            response["robot_info"]["version"]["chassis_sw_ver"] = status.response.chassis_sw_ver;
            response["robot_info"]["version"]["chassis_core_hw_ver"] = status.response.chassis_core_hw_ver;
            response["robot_info"]["version"]["chassis_base_hw_ver"] = status.response.chassis_base_hw_ver;
            response["robot_info"]["version"]["monitor_sw_ver"] = status.response.monitor_sw_ver;
            response["robot_info"]["version"]["monitor_core_hw_ver"] = status.response.monitor_core_hw_ver;
            response["robot_info"]["version"]["monitor_base_hw_ver"] = status.response.monitor_base_hw_ver;
            response["robot_info"]["version"]["bms_hw_ver"] = status.response.bms_hw_ver;
            response["robot_info"]["version"]["bms_sw_ver"] = status.response.bms_sw_ver;
        }
    }

    {
      shm::UpgradeStatus upgrade_status;
      if (shm::iMemory_read_UpgradeStatus(&upgrade_status)) {
          response["robot_info"]["upgrade"]["status"] = upgrade_status.status;
          response["robot_info"]["upgrade"]["upgrading"] = upgrade_status.upgrading;
      }
    }

    // odom info
    #if 0
    {
        atris_msgs::GetChassisInfo info;
        if (get_chassis_info_srv_client_.call(info)) {
            response["robot_info"]["odom"]["odo"]  = info.response.odo;
            response["robot_info"]["odom"]["speed_linear"]  = info.response.speed_linear;
            response["robot_info"]["odom"]["speed_theta"]  = info.response.speed_theta;
        }
    }
    #endif
    //getOdomInfo(response);

    //getImuError(response);

    response["id"] = id;
    response["timestamp"] = timestamp;
    response["result"] = "success";
    return response;
}

void Diagnostics::requestDiagnosticsRobotInfo(const atris_msgs::SignalMessage& msg, const Json::Value& root) {
    Json::Value response = diagnosticsRobotInfo(root["content"]["id"].asString(), root["content"]["timestamp"].asInt64());
    Utils::get_instance()->responseResult(msg, response, "response_robot_info");
}

// upload x86 logs and chassis controller and system monitor log
void Diagnostics::upload_logs(const atris_msgs::SignalMessage &msg) {
  log_info("*************** %s ***********************\r\n",__FUNCTION__);
  int iRet;
  
  // pack log folders even it is empty
  tinyros::Time now = tinyros::Time::now();
  std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
  std::string file = "/userdata/atris_app/" + uid.str() + ".tar.gz";
  //std::string cmd = "tar -czf " + file + " /userdata/atris_app/logs /userdata/impdata/yunxin /home/atris/atris_app/bin/dist/hfs/upload/mcu_log/chassis_controller /home/atris/atris_app/bin/dist/hfs/upload/mcu_log/monitor";
  //std::string cmd = "tar -czf " + file + " /userdata/atris_app/logs /userdata/impdata/yunxin /home/atris/xs_nav/data /home/atris/xs_nav/log";
  std::string cmd = "tar -czf " + file + " /userdata/atris_app/logs /userdata/impdata/yunxin";
  log_warn("%s now we pack log together cmd : %s", __FUNCTION__, cmd.c_str());

  Json::Reader reader;
  Json::Value root, response;
  reader.parse(msg.msg, root);
  response["id"] = root["content"]["id"];
  response["timestamp"] = root["content"]["timestamp"];
  response["result"] = "success";
  response["url"] = "";

  do {
    FILE *fp = NULL;
    
    if((fp = popen(cmd.c_str(), "r"))) {
      pclose(fp);
    } else {
      log_error("%s pack log failed!", __FUNCTION__);
      response["result"] = "fail_pack_log";
      break;
    }

    boost::shared_ptr<UploadLogFile> upload_file = boost::shared_ptr<UploadLogFile> (new UploadLogFile());
    upload_file->local_path = file;
    if(Config::get_instance()->hfs_type == "qiniu"){
        upload_file->remote_path = QINIU_BUCKET_NAME + uid.str() + ".tar.gz";
    }else{
        if (*(Config::get_instance()->hfs_url.end()-1) != '/') {
            upload_file->remote_path = Config::get_instance()->hfs_url + "/" + uid.str() + ".tar.gz";
        }else{
            upload_file->remote_path = Config::get_instance()->hfs_url + uid.str() + ".tar.gz";
        }
    }

    upload_file->deleteDays = 7;
    upload_file->state = TRANSFER_FILE_STARTED;
    TransferFile::upload(upload_file);
    
    while(true) {
      if(upload_file->state == TRANSFER_FILE_COMPLETED) {
        if(Config::get_instance()->hfs_url.find("upload") != std::string::npos){
            log_debug("use go-fastdfs...");
            upload_file->remote_path = Utils::get_instance()->get_fileurl();
        }
        else if (Config::get_instance()->hfs_url.find("udfs-tracer") != std::string::npos)
        {
            log_debug("use udfs-tracer...");
            upload_file->remote_path = Utils::get_instance()->get_fileurl();
        }

        response["url"] = upload_file->remote_path;
        std::string rm = "rm -rf " + file;
        if((fp = popen(rm.c_str(), "r"))) {
          pclose(fp);
        }
        break;
      } else if (upload_file->state == TRANSFER_FILE_ERROR) {
            log_error("upload log files fail.");
            response["result"] = "fail_upload_log";
        break;
      }
      usleep(500*1000);
    }
  }while(0);

  Utils::get_instance()->responseResult(msg, response, "response_mcb_log");

  log_uploading_ = false;
}

// receive change of robot notify change of control status
void Diagnostics::notifyRobotControlStatus(const atris_msgs::SignalMessage& msg) {
    //log_info("%s msg titile = %s",__FUNCTION__, msg.title.c_str()); // for debug use
    Json::Reader reader;
    Json::Value root, response;
    if (msg.title == "response_switch_ptz_light") 
    {
        reader.parse(msg.msg, root);
        if(!root["content"]["id"].isNull() && !root["content"]["timestamp"].isNull() && !root["content"]["result"].isNull())
        {
            if(root["content"]["result"].asString() == "success")
            {
                notifyPtzStatusToShadow();
            }
            else
            {
                log_warn("%s response control ptz light failed , do not need to notify to shadow",__FUNCTION__);
            }
        }
        else
        {
            log_error("%s invalid response from ptz, cannot parse data",__FUNCTION__);
        }
    }
    else if (msg.title == "response_switch_ptz_wiper")
    {
        reader.parse(msg.msg, root);
        if(!root["content"]["id"].isNull() && !root["content"]["timestamp"].isNull() && !root["content"]["result"].isNull())
        {
            if(root["content"]["result"].asString() == "success")
            {
                notifyPtzStatusToShadow();
            }
            else
            {
                log_warn("%s response control ptz wiper failed , do not need to notify to shadow",__FUNCTION__);
            }
        }
        else
        {
            log_error("%s invalid response from ptz, cannot parse data",__FUNCTION__);
        }
    }
    else if (msg.title == "response_robot_standby_mode")
    {
        reader.parse(msg.msg, root);
        if(!root["content"]["id"].isNull() && !root["content"]["timestamp"].isNull() && !root["content"]["result"].isNull())
        {
            if(root["content"]["result"].asString() == "success")
            {
                notifyStandbyModeToShadow();
            }
            else
            {
                log_warn("%s response control robot into standby mode failed , do not need to notify to shadow",__FUNCTION__);
            }
        }
        else
        {
            log_error("%s invalid response from mcu manager, cannot parse data",__FUNCTION__);
        }
    }
    else if (msg.title == "response_robot_wake_up")
    {
        reader.parse(msg.msg, root);
        if(!root["content"]["id"].isNull() && !root["content"]["timestamp"].isNull() && !root["content"]["result"].isNull())
        {
            if(root["content"]["result"].asString() == "success")
            {
                // wake up from standby mode, reset the system check variable
                system_delay_check_ = false;

                if(pSelfDiagTimer != NULL)
                {
                    delete pSelfDiagTimer;
                    pSelfDiagTimer = new CTimer("self_check_diag_timer");
                    pSelfDiagTimer->AsyncOnce(3* 60 * 1000, &Diagnostics::sysDiagDelayProc, this);
                }

                notifyStandbyModeToShadow();
            }
            else
            {
                log_warn("%s response control robot wake up from standby mode failed , do not need to notify to shadow",__FUNCTION__);
            }
        }
        else
        {
            log_error("%s invalid response from mcu manager, cannot parse data",__FUNCTION__);
        }
    }
    else
    {
        //log_warn("%s unrecognized message title",__FUNCTION__);
    }
}

void Diagnostics::notifyStandbyModeToShadow(void)
{
    atris_msgs::GetStandbyMode status;
    if (get_standby_mode_srv_client_.call(status)) 
    {
        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            //log_info("%s light status : %d",__FUNCTION__, status.response.light_status);
            diag_["robot_info"]["base"]["machine_status"]  = status.response.standby_mode;
            machine_status_ = diag_["robot_info"]["base"]["machine_status"].asInt();
        }
    }

    notifyRobotInfoToShadow();
}

void Diagnostics::notifyPtzStatusToShadow(void)
{
    atris_msgs::GetPtzStatus status;
    requestPtzParam();
    if (get_ptz_status_srv_client_.call(status)) 
    {
        boost::unique_lock<boost::mutex> lock(mutex_);
        log_info("%s light status : %d",__FUNCTION__, status.response.light_status);
        log_info("%s wiper status : %d",__FUNCTION__, status.response.wiper_status);
        diag_["robot_info"]["camera"]["status"]  = status.response.status;
        diag_["robot_info"]["camera"]["pan_angle"] = doubleTrim(status.response.pan_angle);
        diag_["robot_info"]["camera"]["tilt_angle"] = doubleTrim(status.response.tilt_angle);
        diag_["robot_info"]["camera"]["zoom_value"] = doubleTrim(status.response.zoom_value);
        diag_["robot_info"]["camera"]["light_status"] = (status.response.light_status == 1)?0:1;
        diag_["robot_info"]["camera"]["wiper_status"] = status.response.wiper_status;
        diag_["robot_info"]["camera"]["error"] = status.response.error;
    }

    notifyRobotInfoToShadow();
}

void Diagnostics::messageInstantReceive(const atris_msgs::SignalMessage& msg) {
    Json::Reader reader;
    Json::Value root, response;
    atris_msgs::CanData can_data_msgs;
    char rand_str[16] = {0};

    response["id"] = msg.msgID;
    response["timestamp"] = msg.timestamp;
    response["result"] = "success";
    
    if (msg.title == "request_robot_info") {
        reader.parse(msg.msg, root);
        requestDiagnosticsRobotInfo(msg, root);
    } else if (msg.title == "request_mcb_log") {
        if (log_uploading_) {
            reader.parse(msg.msg, root);
            response["result"] = "fail_already_mcb_log";
            Utils::get_instance()->responseResult(msg, response, "response_mcb_log");
        } else {
            if (log_thread_) {
                delete log_thread_;
                log_thread_ = NULL;
            }
            log_uploading_ = true;
            log_thread_ = new boost::thread(boost::bind(&Diagnostics::upload_logs, this, msg));
        }
    } else if (msg.title == "request_set_fan_bottom") {
      #if 0
        reader.parse(msg.msg, root);
        if (!root["content"]["speed"].isNull()) {
            int speed = root["content"]["speed"].asInt();
            if (speed >= 0 && speed <=3) {
                can_data_msgs.channel = 0xf4;
                can_data_msgs.data[0] = 0xf1;
                can_data_msgs.data[1] = speed;
                can_data_msgs.data[2] = speed;
                can_data_msgs.size = 3;
                send_can_cmd_pub_.publish(&can_data_msgs);
            } else {
                response["result"] = "fail_invalid_data";
            }
        } else {
            response["result"] = "fail_invalid_data";
        }
        #endif
        Utils::get_instance()->responseResult(msg, response, "response_set_fan_bottom");
    } else if (msg.title == "request_set_fan_middle") {
        atris_msgs::MiddleFanControl middle_fan_ctrl_msg_;
        reader.parse(msg.msg, root);
        if (!getVopCallingStatus() && !root["content"]["speed"].isNull()) {
            int speed = root["content"]["speed"].asInt();
            if (speed >= 0 && speed <=3) {
                #if 0
                can_data_msgs.channel = 0x81;
                can_data_msgs.data[0] = 0x39;
                can_data_msgs.data[1] = speed;
                can_data_msgs.data[2] = speed;
                can_data_msgs.size = 3;
                middle_fan_enable_ = speed > 0 ? true : false;
                send_can_cmd_pub_.publish(&can_data_msgs);
                #endif
                middle_fan_ctrl_msg_.atris_type = atris_msgs::MiddleFanControl::ATRIS_TYPE_OTHER;
                middle_fan_ctrl_msg_.on_off = atris_msgs::MiddleFanControl::MIDDLE_FAN_ON;
                middle_fan_ctrl_msg_.level = speed;
                middle_fan_enable_ = speed > 0 ? true : false;
                middle_fan_ctrl_pub_.publish(middle_fan_ctrl_msg_);

            } else {
                response["result"] = "fail_invalid_data";
            }
        } else {
            response["result"] = "fail_invalid_data";
        }
        Utils::get_instance()->responseResult(msg, response, "response_set_fan_middle");
    } else if (msg.title == "notice_voip_call_established") {
       //voip_calling_status_ = 1;
      log_info("%s voip call connection established",__FUNCTION__);
      //notifyTelecomEvent(1);
      setVoipCallingStatus(1);

      atris_msgs::MiddleFanControl middle_fan_ctrl_msg_;
      #if (defined _POLICE_) && (defined _CHASSIS_MARSHELL_)
      // fan off and police
      middle_fan_ctrl_msg_.on_off = atris_msgs::MiddleFanControl::MIDDLE_FAN_OFF;
      middle_fan_ctrl_msg_.atris_type = atris_msgs::MiddleFanControl::ATRIS_TYPE_M_POLICE;
      middle_fan_ctrl_pub_.publish(middle_fan_ctrl_msg_);
      // no level for police version

      #elif (defined _CHASSIS_MARSHELL_)
      // fan off and standard mashell
      middle_fan_ctrl_msg_.on_off = atris_msgs::MiddleFanControl::MIDDLE_FAN_OFF;
      middle_fan_ctrl_msg_.atris_type = atris_msgs::MiddleFanControl::ATRIS_TYPE_OTHER;
      middle_fan_ctrl_msg_.level = 0;
      middle_fan_ctrl_pub_.publish(middle_fan_ctrl_msg_);

      #else
      // no action for jc version
      #endif

    } else if (msg.title == "notice_voip_call_terminated") {
       //voip_calling_status_ = 0;
      log_info("%s voip call connection terminated",__FUNCTION__);
      //notifyTelecomEvent(0);
      setVoipCallingStatus(0);
      atris_msgs::MiddleFanControl middle_fan_ctrl_msg_;
    } else if (msg.title == "request_4g_state") {
      log_info("receive 4g state query");
      Get4gState(response);
      Utils::get_instance()->responseResult(msg, response, "response_4g_state");
    } else if (msg.title == "request_background_control") {
      log_info("%s request background control",__FUNCTION__);
      reader.parse(msg.msg, root);
      int background_ctrl_status;
      Json::FastWriter jwriter;
      std::string background_control_content = ""; // for debug purpose
      background_control_content = jwriter.write(root);
      log_info("%s background control content : %s", __FUNCTION__, background_control_content.c_str());
      
      if(!root["content"].isNull() && !root["content"]["id"].isNull() && !root["content"]["timestamp"].isNull() && !root["content"]["status"].isNull())
      {
        background_ctrl_status = root["content"]["status"].asInt();
        log_warn("&&&&&&&&&&&&& %s background control status = %d", __FUNCTION__, background_ctrl_status);
        response["result"] = "success";
        response["status"] = background_ctrl_status;
        setRobotRunMode(3,(uint8_t)(background_ctrl_status));
        Utils::get_instance()->responseResult(msg, response, "response_background_control");
      }
      else
      {
        response["status"] = 0;
        response["result"] = "fail_invalid_data";
        Utils::get_instance()->responseResult(msg, response, "response_background_control");
      }
    }
    else if(msg.title == "request_set_robot_warning")
    {
        // for robot warning simulation
        reader.parse(msg.msg, root);
        //resp_name = "response_set_robot_warning";
        int type;
        int on_off;
        if (root["content"]["type"].isNull() || root["content"]["on_off"].isNull()) 
        {
            log_error("%s request set robot warning , type field or on_off field is NULL",__FUNCTION__);
            response["result"] = "fail_invalid_data";
        } 
        else
        {
            /*
            TODO 
            */
            type = root["content"]["type"].asInt();
            on_off = root["content"]["on_off"].asInt();
            log_info("%s request set robot warning, type = %d, on_off=%d",__FUNCTION__, type, on_off);
            switch(type)
            {
                case 1:
                    //bms error
                    checkBMSErrorEvent(on_off);
                break;
                case 2:
                    // chassis driver error
                    checkChassisErrorEvent(on_off);
                break;
                case 3:
                    // device error(ptz error)
                    checkPTZErrorEvent(on_off);
                break;
                case 4:
                    //ccb error
                    checkCCBErrorEvent(on_off);
                break;
                case 5:
                    //sys error
                    checkSYSErrorEvent(on_off);
                break;
        
                default:
                    log_error("%s unrecognized type : %d", __FUNCTION__, type);
                break;
            }

            response["result"] = "success";
        }

        Utils::get_instance()->responseResult(msg, response, "response_set_robot_warning");
    }
    else if(msg.title == "request_set_robot_event_simulation")
    {
        // for robot warning simulation switch use
        reader.parse(msg.msg, root);

        if (root["content"]["on_off"].isNull()) 
        {
            log_error("%s request set robot event simulation , type on_off field is NULL",__FUNCTION__);
            response["result"] = "fail_invalid_data";
        } 
        else
        {
            simulation_event_switch_ = root["content"]["on_off"].asInt();
            log_info("%s event simulation switch is set to %d\r\n",__FUNCTION__, simulation_event_switch_);
            response["result"] = "success";
        }

        Utils::get_instance()->responseResult(msg, response, "response_set_robot_event_simulation");
    }
}

void Diagnostics::Get4gState(Json::Value & tmp)
{
  Json::Value temp = getValue();
  if(!temp["robot_info"].isNull() && !temp["robot_info"]["4g"].isNull() && !temp["robot_info"]["4g"]["level"].isNull() && !temp["robot_info"]["4g"]["error"].isNull())
  {
    log_info("%s level = %d , error = %d\r\n",__FUNCTION__, temp["robot_info"]["4g"]["level"].asInt(), temp["robot_info"]["4g"]["error"].asInt());
    tmp["value"] = temp["robot_info"]["4g"]["level"].asInt();
    tmp["error"] = temp["robot_info"]["4g"]["error"].asInt();
  }
  else
  {
    tmp["value"] = 0;
    tmp["error"] = -1;
    tmp["result"] = "fail_invalid_data";
  }
}

void Diagnostics::checkRobotCollision()
{
    Json::Value tmp = diag_;
    Json::Value root;
    int bumper_state_this_ = -1;
    int bumper_trig_source = BUMPER_TRIGGER_NULL;
    static std::string bumper_state_serial_num_ = "";
    double now = tinyros::Time().now().toSec();

    if (!tmp["robot_info"].isNull() 
        && !tmp["robot_info"]["brake"].isNull()
        && !tmp["robot_info"]["brake"]["source"].isNull()
        && !tmp["robot_info"]["brake"]["source"]["front_bumper"].isNull()
        && !tmp["robot_info"]["brake"]["source"]["tear_bumper"].isNull())
    {
        if((tmp["robot_info"]["brake"]["source"]["front_bumper"].asInt() == 0 || tmp["robot_info"]["brake"]["source"]["front_bumper"].asInt() == 1)
            && (tmp["robot_info"]["brake"]["source"]["tear_bumper"].asInt() == 0 || tmp["robot_info"]["brake"]["source"]["tear_bumper"].asInt() == 1))
        {

            //log_info("%s front bumper : %d , tear bumper : %d",__FUNCTION__, tmp["robot_info"]["brake"]["source"]["front_bumper"].asInt(),tmp["robot_info"]["brake"]["source"]["tear_bumper"].asInt());
            if(tmp["robot_info"]["brake"]["source"]["front_bumper"].asInt() == 1 || tmp["robot_info"]["brake"]["source"]["tear_bumper"].asInt() == 1)
            {
                bumper_state_this_ = 1;
            }
            else if(tmp["robot_info"]["brake"]["source"]["front_bumper"].asInt() == 0 && tmp["robot_info"]["brake"]["source"]["tear_bumper"].asInt() == 0)
            {
                bumper_state_this_ = 0;
            }
            else
            {
                log_error("%s unknown case!!!\r\n",__FUNCTION__);
            }
        }
        else
        {
            log_warn("%s front bumper or tear bumper data value feild invalid!!!",__FUNCTION__);
            return;
        }
    }
    else
    {
        log_warn("%s front bumper or tear bumper data invalid",__FUNCTION__);
        return;
    }

    if(bumper_state_last_ == 0 && bumper_state_this_ == 1)
    {
        log_warn("%s robot collision triggerd please check!!\r\n",__FUNCTION__);
        //robot_collision = true;
        //bumper_trig_sound_last_ = now;
        // report robot collision event

        if(tmp["robot_info"]["brake"]["source"]["front_bumper"].asInt()==1)
        {
            bumper_trig_source = BUMPER_TRIGGER_FRONT;
        }
        else if(tmp["robot_info"]["brake"]["source"]["tear_bumper"].asInt()==1)
        {
            bumper_trig_source = BUMPER_TRIGGER_TEAR;
        }
        else
        {
            bumper_trig_source = BUMPER_TRIGGER_NULL;
        }

        log_warn("%s bumper trigger source = %d",__FUNCTION__, bumper_trig_source);
    }
    else if(bumper_state_last_ == 1 && bumper_state_this_ == 0)
    {
        log_warn("%s robot collision triggerd recovered!!\r\n",__FUNCTION__);
    }
    else
    {

    }

    bumper_state_last_ = bumper_state_this_;
}

void Diagnostics::requestPtzParam(void)
{
    log_info("%s",__FUNCTION__);
    Json::FastWriter jwriter;
    Json::Value root;
    atris_msgs::SignalMessage ptz_msg;
    std::string ptz_msg_content = ""; // for debug purpose
    tinyros::Time now = tinyros::Time::now();
    std::stringstream uid; 
    uid << ((uint64_t) (now.toSec() * 1000000000ull));

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    root["title"] = "request_ptz_param";
    root["accid"] = shmrbt.robot.sn;
    root["content"]["id"] = Json::Value(uid.str());
    root["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000); // in ms
    ptz_msg_content = jwriter.write(root);
    
    ptz_msg.title = "request_ptz_param";
    ptz_msg.msg = jwriter.write(root);
    ptz_msg.msgID = uid.str();
    ptz_msg.timestamp = root["content"]["timestamp"].asInt64();
    recharge_battery_pub_.publish(ptz_msg);
}

#if 0
void Diagnostics::checkRobotEmergeBrakeEvent()
{
    int button_state_this_ = 0;
    int can_state_this_ = 0;
    static int emerge_brake_status_ = 0;
    static int button_state_last_ = 0;
    static int can_state_last_ = 0;
	  static std::string emerge_brake_serial_num_ = "";
    int emerge_brake_status_this_ = 0;
    //int emerge_brake_trig_source_ = EMERGE_BREAK_NULL;
    Json::Value tmp = diag_;
    Json::Value root;
    char rand_str[16] = {0};

    if (!tmp["robot_info"].isNull() 
        && !tmp["robot_info"]["brake"].isNull()
        && !tmp["robot_info"]["brake"]["source"].isNull()
        && !tmp["robot_info"]["brake"]["source"]["button"].isNull()
        && !tmp["robot_info"]["brake"]["source"]["can"].isNull()
        )
    {
        // check if data valid
        if((tmp["robot_info"]["brake"]["source"]["button"].asInt() == 1 || tmp["robot_info"]["brake"]["source"]["button"].asInt() == 0)
            &&(tmp["robot_info"]["brake"]["source"]["can"].asInt() == 1 || tmp["robot_info"]["brake"]["source"]["can"].asInt() == 0)
            )
        {
            button_state_this_ = tmp["robot_info"]["brake"]["source"]["button"].asInt();
            can_state_this_ = tmp["robot_info"]["brake"]["source"]["can"].asInt();
            log_info("%s button value : %d , can value : %d\r\n",__FUNCTION__, button_state_this_, can_state_this_);

            if(button_state_this_ == 1 || can_state_this_ == 1)
            {
                emerge_brake_status_this_ = 1;
            }
            else if(button_state_this_ == 0 && can_state_this_ == 0)
            {
                emerge_brake_status_this_ = 0;
            }


            if(button_state_this_ == 1 && button_state_last_ == 0 && emerge_brake_status_this_ == 1)
            {
                Utils::get_instance()->getRandStr(rand_str);
                emerge_brake_serial_num_ = rand_str;
                log_warn("****************emergency brake triggered , source is button*********************");
            }


            if(can_state_this_ == 1 && can_state_last_ == 0 && emerge_brake_status_this_ == 1)
            {
                log_warn("****************emergency brake triggered , source is can*********************");
                Utils::get_instance()->getRandStr(rand_str);
                emerge_brake_serial_num_ = rand_str;

            }


            if(emerge_brake_status_this_ == 0 && emerge_brake_status_ == 1)
            {
                log_warn("****************emergency brake triggered is NULL*********************");
            }
            else
            {

            }

            log_info("before -------> emerge_brake_status_ = %d, emerge_brake_status_this_ = %d", emerge_brake_status_, emerge_brake_status_this_);
            log_info("before -------> button_state_last_ = %d, button_state_this_ = %d", button_state_last_, button_state_this_);
            log_info("before -------> can_state_last_ = %d, can_state_this_ = %d", can_state_last_, can_state_this_);

            emerge_brake_status_ = emerge_brake_status_this_;
            button_state_last_ = button_state_this_;
            can_state_last_ = can_state_this_;

            log_info("after -------> emerge_brake_status_ = %d, emerge_brake_status_this_ = %d", emerge_brake_status_, emerge_brake_status_this_);
            log_info("after -------> button_state_last_ = %d, button_state_this_ = %d", button_state_last_, button_state_this_);
            log_info("after -------> can_state_last_ = %d, can_state_this_ = %d", can_state_last_, can_state_this_);
            
        }

    }
}
#endif

// check emergency button when start up
void Diagnostics::checkEmergeButtonWhenStartUp(int button_status)
{
    if(!(button_status == 0 || button_status == 1))
    {
        log_error("%s button status value invalid , button_status = %d", __FUNCTION__, button_status);
        return;
    }
    // this function only can be called once
    static int check_emerge_button_event_flag = 0;
    int lidar_status;
    DataBaseCmdStru button_status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();

    if(!(button_status == 0 || button_status == 1))
    {
        log_error("%s button status value invalid , button_status = %d", __FUNCTION__, button_status);
        return;
    }

    if(!check_emerge_button_event_flag)
    {
        check_emerge_button_event_flag = 1;
        // do not need serial number in this function , it just check if it is recovered
        log_warn("^^^^^^^^^^^^^^^^^^^^^^^^ %s check emerge button status if it is recovered", __FUNCTION__);
        log_warn("^^^^^^^^^^^^^^^^^^^^^^^^ %s function only be called once", __FUNCTION__);
        button_status_cmd_stru.cmd_type = EVENT_CHECK_LAST_CYCLE;
        button_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        button_status_cmd_stru.event_data.data_item.event_status = button_status;
        button_status_cmd_stru.event_data.data_item.event_content = "emerge_stop_warnning"; // check lidar erro event content field
        button_status_cmd_stru.event_data.data_item.event_serial_num = ""; // deliberately empty

        diagEventCmdAdd(button_status_cmd_stru);

    }
}

void Diagnostics::checkEmergeButtonEvent(int button_status)
{
    //log_info("%s",__FUNCTION__);
    int button_status_this_;
    static std::string emerge_button_serial_num_ = "";
    char rand_str[16] = {0};
    tinyros::Time now = tinyros::Time::now();
    DataBaseCmdStru button_status_cmd_stru;

    button_status_this_ = button_status;
    //
    if(!(button_status == 0 || button_status == 1))
    {
        log_error("%s button status value invalid , button_status = %d", __FUNCTION__, button_status);
        return;
    }

    if(button_status_this_ == 1 && emerge_stop_warning_stats_ == 0)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! emergency button pushed this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        Utils::get_instance()->getRandStr(rand_str);
        emerge_button_serial_num_ = rand_str;
        notifyAbnormalWarningEvent("emerge_stop_warnning", button_status_this_, (uint64_t)now.toSec(), emerge_button_serial_num_);
        button_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        button_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        button_status_cmd_stru.event_data.data_item.event_status = button_status_this_;
        button_status_cmd_stru.event_data.data_item.event_content = "emerge_stop_warnning"; // check lidar erro event content field
        button_status_cmd_stru.event_data.data_item.event_serial_num = emerge_button_serial_num_; // deliberately empty
        // save it to database
        diagEventCmdAdd(button_status_cmd_stru);
         
    }
    else if(button_status_this_ == 0 && emerge_stop_warning_stats_ == 1)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! emergency button released this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        notifyAbnormalWarningEvent("emerge_stop_warnning", button_status_this_, (uint64_t)now.toSec(), emerge_button_serial_num_);
        button_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        button_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        button_status_cmd_stru.event_data.data_item.event_status = button_status_this_;
        button_status_cmd_stru.event_data.data_item.event_content = "emerge_stop_warnning"; // check lidar erro event content field
        button_status_cmd_stru.event_data.data_item.event_serial_num = emerge_button_serial_num_; // deliberately empty
        diagEventCmdAdd(button_status_cmd_stru);
    }
    else
    {
        //log_info("no change to emerge button status = %d", button_status_this_);
    }

    emerge_stop_warning_stats_ = button_status_this_;
}
     

void Diagnostics::checkBMSWhenStartUp(int bms_error_status)
{
    //log_info("%s ",__FUNCTION__);
    static int check_bms_event_flag = 0;
    DataBaseCmdStru bms_status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();

    if(!check_bms_event_flag)
    {
        check_bms_event_flag = 1;
        // do not need serial number in this function , it just check if it is recovered
        log_warn("%s check bms status if it is recovered", __FUNCTION__);
        bms_status_cmd_stru.cmd_type = EVENT_CHECK_LAST_CYCLE;
        bms_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        bms_status_cmd_stru.event_data.data_item.event_status = bms_error_status;
        bms_status_cmd_stru.event_data.data_item.event_content = "battery_error"; // check lidar erro event content field
        bms_status_cmd_stru.event_data.data_item.event_serial_num = ""; // deliberately empty

        diagEventCmdAdd(bms_status_cmd_stru);
    }
}


// normal check bms event
// only update the database when there is status changed to the device error status
void Diagnostics::checkBMSErrorEvent(int bms_error_status)
{

    char rand_str[16] = {0};
    tinyros::Time now = tinyros::Time::now();
    DataBaseCmdStru bms_status_cmd_stru;
    std::string bms_serial_num;

    if(bms_error_status == 1 && battery_error_stats_ == 0)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! bms error occured this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        sendDeviceWarning(1);
        Utils::get_instance()->getRandStr(rand_str);
        bms_serial_num = rand_str;
        setBmsSerialNum(bms_serial_num);

        notifyDeviceWarningEvent("battery_error", bms_error_status, (uint64_t)now.toSec(), bms_serial_num);
        bms_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        bms_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        bms_status_cmd_stru.event_data.data_item.event_status = bms_error_status;
        bms_status_cmd_stru.event_data.data_item.event_content = "battery_error"; // check bms error event content field
        bms_status_cmd_stru.event_data.data_item.event_serial_num = bms_serial_num; // deliberately empty
        // save it to database
        diagEventCmdAdd(bms_status_cmd_stru);
         
    }
    else if(bms_error_status == 0 && battery_error_stats_ == 1)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! bms error recovered this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        bms_serial_num = getBmsSerialNum();
        sendDeviceWarning(0);
        notifyDeviceWarningEvent("battery_error", bms_error_status, (uint64_t)now.toSec(), bms_serial_num);
        bms_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        bms_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        bms_status_cmd_stru.event_data.data_item.event_status = bms_error_status;
        bms_status_cmd_stru.event_data.data_item.event_content = "battery_error"; // check bms error event content field
        bms_status_cmd_stru.event_data.data_item.event_serial_num = bms_serial_num; // deliberately empty
        diagEventCmdAdd(bms_status_cmd_stru);
    }
    else
    {
        //log_info("no change to bms error status bms error status = %d", bms_error_status);
    }

    battery_error_stats_ = bms_error_status;
    
}

void Diagnostics::checkChassisWhenStartUp(int chassis_error_status)
{
    static int check_chassis_event_flag = 0;
    //int chassis_error_status;
    DataBaseCmdStru chassis_status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();
    #if 0
    if(bms_communication_ok_ == 1 || isBmsCharging_ || (machine_status_ == 1))
    {
        //log_warn("%s bms status not ready or bms is charging, skip check chassis error",__FUNCTION__);
        return;
    }

    if(chassis_lf_error_stats_==-1||chassis_rf_error_stats_==-1||chassis_lr_error_stats_==-1||chassis_rr_error_stats_==-1)
    {
        //log_error("%s chassis four wheel error info collected not completed",__FUNCTION__);
        return;
    }

    if(chassis_lf_error_stats_ == 1||chassis_rf_error_stats_==1||chassis_lr_error_stats_==1||chassis_rr_error_stats_==1)
    {
        chassis_error_status = 1;
    }
    else
    {
        chassis_error_status = 0;
    }
    #endif

    if(!check_chassis_event_flag)
    {
        check_chassis_event_flag = 1;
        // do not need serial number in this function , it just check if it is recovered
        log_warn("%s check chassis status if it is recovered", __FUNCTION__);
        chassis_status_cmd_stru.cmd_type = EVENT_CHECK_LAST_CYCLE;
        chassis_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        chassis_status_cmd_stru.event_data.data_item.event_status = chassis_error_status;
        chassis_status_cmd_stru.event_data.data_item.event_content = "chassis_error"; // check lidar erro event content field
        chassis_status_cmd_stru.event_data.data_item.event_serial_num = ""; // deliberately empty

        diagEventCmdAdd(chassis_status_cmd_stru);
    }
}

void Diagnostics::checkChassisErrorEvent(int chassis_error_status)
{
    //static int chassis_control_error_last_ = 0;
    
    int chassis_driver_error_this = -1;
    Json::Value root;
    char rand_str[16] = {0};
    tinyros::Time now = tinyros::Time::now();
    DataBaseCmdStru chassis_status_cmd_stru;
    std::string chassis_serial_num;
    #if 0

    if(chassis_lf_error_stats_==-1||chassis_rf_error_stats_==-1||chassis_lr_error_stats_==-1||chassis_rr_error_stats_==-1)
    {
        //log_error("%s chassis four wheel error info collected not completed",__FUNCTION__); // just return
        return;
    }

    if(bms_communication_ok_ != 1 || isBmsCharging_ || (machine_status_ == 1))
    {
        //log_warn("%s bms status not ready or bms is charging, skip check chassis error",__FUNCTION__);
        return;
    }

    if(chassis_lf_error_stats_ == 1||chassis_rf_error_stats_==1||chassis_lr_error_stats_==1||chassis_rr_error_stats_==1)
    {
        chassis_driver_error_this = 1;
    }
    else
    {
        chassis_driver_error_this = 0;
    }
    #endif

    if(1 == chassis_error_status && 0 == chassis_error_stats_)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! chassis error occured this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        sendDeviceWarning(1);
        Utils::get_instance()->getRandStr(rand_str);
        chassis_serial_num = rand_str;
        setChassisSerialNum(chassis_serial_num);
        notifyDeviceWarningEvent("chassis_error", chassis_driver_error_this, (uint64_t)now.toSec(), chassis_serial_num);
        chassis_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        chassis_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        chassis_status_cmd_stru.event_data.data_item.event_status = chassis_driver_error_this;
        chassis_status_cmd_stru.event_data.data_item.event_content = "chassis_error"; // check ptz erro event content field
        chassis_status_cmd_stru.event_data.data_item.event_serial_num = chassis_serial_num; // deliberately empty
        // save it to database
        diagEventCmdAdd(chassis_status_cmd_stru);

    }
    else if(0 == chassis_driver_error_this && 1 == chassis_error_stats_)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! chassis error recovered this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        chassis_serial_num = getChassisSerialNum();
        sendDeviceWarning(0);
        notifyDeviceWarningEvent("chassis_error", chassis_driver_error_this, (uint64_t)now.toSec(), chassis_serial_num);
        chassis_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        chassis_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        chassis_status_cmd_stru.event_data.data_item.event_status = chassis_driver_error_this;
        chassis_status_cmd_stru.event_data.data_item.event_content = "chassis_error"; // check ptz erro event content field
        chassis_status_cmd_stru.event_data.data_item.event_serial_num = chassis_serial_num; // deliberately empty
        diagEventCmdAdd(chassis_status_cmd_stru);
    }
    else
    {

    }

    chassis_error_stats_ = chassis_driver_error_this;
}

// check ptz error 3 minutes after system up( system_delay_check_ )
void Diagnostics::checkPTZWhenStartUp(int ptz_error_status)
{
    //log_info("%s ",__FUNCTION__);
    static int check_ptz_event_flag = 0;
    DataBaseCmdStru ptz_status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();

    if(!check_ptz_event_flag)
    {
        check_ptz_event_flag = 1;
        // do not need serial number in this function , it just check if it is recovered
        log_warn("%s check ptz status if it is recovered", __FUNCTION__);
        //log_warn("%s function only be called once", __FUNCTION__);
        ptz_status_cmd_stru.cmd_type = EVENT_CHECK_LAST_CYCLE;
        ptz_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        ptz_status_cmd_stru.event_data.data_item.event_status = ptz_error_status;
        ptz_status_cmd_stru.event_data.data_item.event_content = "ptz_error"; // check ptz error event content field
        ptz_status_cmd_stru.event_data.data_item.event_serial_num = ""; // deliberately empty

        diagEventCmdAdd(ptz_status_cmd_stru);
    }

}

void Diagnostics::checkCCBWhenStartUp(int ccb_error_status)
{
    //log_info("%s ",__FUNCTION__);
    static int check_ccb_event_flag = 0;
    DataBaseCmdStru ccb_status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();

    if(!check_ccb_event_flag)
    {
        check_ccb_event_flag = 1;
        // do not need serial number in this function , it just check if it is recovered
        log_warn("%s check ccb status if it is recovered", __FUNCTION__);
        //log_warn("%s function only be called once", __FUNCTION__);
        ccb_status_cmd_stru.cmd_type = EVENT_CHECK_LAST_CYCLE;
        ccb_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        ccb_status_cmd_stru.event_data.data_item.event_status = ccb_error_status;
        ccb_status_cmd_stru.event_data.data_item.event_content = "ccb_error"; // check ccb error event content field
        ccb_status_cmd_stru.event_data.data_item.event_serial_num = ""; // deliberately empty

        diagEventCmdAdd(ccb_status_cmd_stru);
    }

}

void Diagnostics::checkSYSWhenStartUp(int sys_error_status)
{
    static int check_sys_event_flag = 0;
    DataBaseCmdStru sys_status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();

    if(!check_sys_event_flag)
    {
        check_sys_event_flag = 1;
        // do not need serial number in this function , it just check if it is recovered
        log_warn("%s check sys status if it is recovered", __FUNCTION__);
        //log_warn("%s function only be called once", __FUNCTION__);
        sys_status_cmd_stru.cmd_type = EVENT_CHECK_LAST_CYCLE;
        sys_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        sys_status_cmd_stru.event_data.data_item.event_status = sys_error_status;
        sys_status_cmd_stru.event_data.data_item.event_content = "sys_error"; // check sys error event content field
        sys_status_cmd_stru.event_data.data_item.event_serial_num = ""; // deliberately empty

        diagEventCmdAdd(sys_status_cmd_stru);
    }

}

void Diagnostics::checkCCBErrorEvent(int ccb_error_status)
{

    char rand_str[16] = {0};
    tinyros::Time now = tinyros::Time::now();
    DataBaseCmdStru ccb_status_cmd_stru;
    std::string ccb_serial_num;

    if(ccb_error_status == 1 && ccb_error_stats_ == 0)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! ccb error occured this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        sendDeviceWarning(1);
        Utils::get_instance()->getRandStr(rand_str);
        ccb_serial_num = rand_str;
        setCCBSerialNum(ccb_serial_num);

        notifyDeviceWarningEvent("ccb_error", ccb_error_status, (uint64_t)now.toSec(), ccb_serial_num);
        ccb_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        ccb_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        ccb_status_cmd_stru.event_data.data_item.event_status = ccb_error_status;
        ccb_status_cmd_stru.event_data.data_item.event_content = "ccb_error"; // check ccb erro event content field
        ccb_status_cmd_stru.event_data.data_item.event_serial_num = ccb_serial_num; // deliberately empty
        // save it to database
        diagEventCmdAdd(ccb_status_cmd_stru);
         
    }
    else if(ccb_error_status == 0 && ccb_error_stats_ == 1)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! ccb error recovered this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        ccb_serial_num = getCCBSerialNum();
        sendDeviceWarning(0);
        notifyDeviceWarningEvent("ccb_error", ccb_error_status, (uint64_t)now.toSec(), ccb_serial_num);
        ccb_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        ccb_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        ccb_status_cmd_stru.event_data.data_item.event_status = ccb_error_status;
        ccb_status_cmd_stru.event_data.data_item.event_content = "ccb_error"; // check ccb erro event content field
        ccb_status_cmd_stru.event_data.data_item.event_serial_num = ccb_serial_num; // deliberately empty
        diagEventCmdAdd(ccb_status_cmd_stru);
    }
    else
    {
        // temporily doing nothing
    }

    ccb_error_stats_ = ccb_error_status;
    
}

void Diagnostics::checkSYSErrorEvent(int sys_error_status)
{

    char rand_str[16] = {0};
    tinyros::Time now = tinyros::Time::now();
    DataBaseCmdStru sys_status_cmd_stru;
    std::string sys_serial_num;

    if(sys_error_status == 1 && sys_error_stats_ == 0)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! sys error occured this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        sendDeviceWarning(1);
        Utils::get_instance()->getRandStr(rand_str);
        sys_serial_num = rand_str;
        setSYSSerialNum(sys_serial_num);

        notifyDeviceWarningEvent("sys_error", sys_error_status, (uint64_t)now.toSec(), sys_serial_num);
        sys_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        sys_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        sys_status_cmd_stru.event_data.data_item.event_status = sys_error_status;
        sys_status_cmd_stru.event_data.data_item.event_content = "sys_error"; // check sys erro event content field
        sys_status_cmd_stru.event_data.data_item.event_serial_num = sys_serial_num; // deliberately empty
        // save it to database
        diagEventCmdAdd(sys_status_cmd_stru);
         
    }
    else if(sys_error_status == 0 && sys_error_stats_ == 1)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! sys error recovered this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        sys_serial_num = getSYSSerialNum();
        sendDeviceWarning(0);
        notifyDeviceWarningEvent("sys_error", sys_error_status, (uint64_t)now.toSec(), sys_serial_num);
        sys_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        sys_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        sys_status_cmd_stru.event_data.data_item.event_status = sys_error_status;
        sys_status_cmd_stru.event_data.data_item.event_content = "sys_error"; // check sys erro event content field
        sys_status_cmd_stru.event_data.data_item.event_serial_num = sys_serial_num; // deliberately empty
        diagEventCmdAdd(sys_status_cmd_stru);
    }
    else
    {
        // temporily doing nothing
    }

    sys_error_stats_ = sys_error_status;
    
}

void Diagnostics::checkLidarWhenStartUp(int lidar_error_status)
{
    static int check_lidar_event_flag = 0;
    DataBaseCmdStru lidar_status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();

    if(!check_lidar_event_flag)
    {
        check_lidar_event_flag = 1;
        // do not need serial number in this function , it just check if it is recovered
        log_warn("%s check lidar status if it is recovered", __FUNCTION__);
        //log_warn("%s function only be called once", __FUNCTION__);
        lidar_status_cmd_stru.cmd_type = EVENT_CHECK_LAST_CYCLE;
        lidar_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        lidar_status_cmd_stru.event_data.data_item.event_status = lidar_error_status;
        lidar_status_cmd_stru.event_data.data_item.event_content = "lidar_error"; // check lidar erro event content field
        lidar_status_cmd_stru.event_data.data_item.event_serial_num = ""; // deliberately empty

        diagEventCmdAdd(lidar_status_cmd_stru);
    }
}

// check ptz error 3 minutes after system up( system_delay_check_ )

void Diagnostics::checkPTZErrorEvent(int ptz_error_status)
{

    char rand_str[16] = {0};
    tinyros::Time now = tinyros::Time::now();
    DataBaseCmdStru ptz_status_cmd_stru;
    std::string ptz_serial_num;

    if(ptz_error_status == 1 && ptz_error_stats_ == 0)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! ptz error occured this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        sendDeviceWarning(1);
        Utils::get_instance()->getRandStr(rand_str);
        ptz_serial_num = rand_str;
        setPTZSerialNum(ptz_serial_num);

        notifyDeviceWarningEvent("ptz_error", ptz_error_status, (uint64_t)now.toSec(), ptz_serial_num);
        ptz_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        ptz_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        ptz_status_cmd_stru.event_data.data_item.event_status = ptz_error_status;
        ptz_status_cmd_stru.event_data.data_item.event_content = "ptz_error"; // check ptz erro event content field
        ptz_status_cmd_stru.event_data.data_item.event_serial_num = ptz_serial_num; // deliberately empty
        // save it to database
        diagEventCmdAdd(ptz_status_cmd_stru);
         
    }
    else if(ptz_error_status == 0 && ptz_error_stats_ == 1)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! ptz error recovered this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        ptz_serial_num = getPTZSerialNum();
        sendDeviceWarning(0);
        notifyDeviceWarningEvent("ptz_error", ptz_error_status, (uint64_t)now.toSec(), ptz_serial_num);
        ptz_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        ptz_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        ptz_status_cmd_stru.event_data.data_item.event_status = ptz_error_status;
        ptz_status_cmd_stru.event_data.data_item.event_content = "ptz_error"; // check ptz erro event content field
        ptz_status_cmd_stru.event_data.data_item.event_serial_num = ptz_serial_num; // deliberately empty
        diagEventCmdAdd(ptz_status_cmd_stru);
    }
    else
    {
        // temporily doing nothing
    }

    ptz_error_stats_ = ptz_error_status;
    
}

// normal check lidar event
// only update the database when there is status changed to the device error status
void Diagnostics::checkLidarErrorEvent(int lidar_error_status)
{
    //log_info("%s",__FUNCTION__);
    static std::string main_lidar_serial_num_ = "";
    char rand_str[16] = {0};
    tinyros::Time now = tinyros::Time::now();
    DataBaseCmdStru lidar_status_cmd_stru;

    if(lidar_error_status == 1 && lidar_error_stats_ == 0)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! lidar error occured this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        sendDeviceWarning(1);
        Utils::get_instance()->getRandStr(rand_str);
        main_lidar_serial_num_ = rand_str;

        notifyDeviceWarningEvent("lidar_error", lidar_error_status, (uint64_t)now.toSec(), main_lidar_serial_num_);
        lidar_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        lidar_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        lidar_status_cmd_stru.event_data.data_item.event_status = lidar_error_status;
        lidar_status_cmd_stru.event_data.data_item.event_content = "lidar_error"; // check lidar erro event content field
        lidar_status_cmd_stru.event_data.data_item.event_serial_num = main_lidar_serial_num_; // deliberately empty
        // save it to database
        diagEventCmdAdd(lidar_status_cmd_stru);
         
    }
    else if(lidar_error_status == 0 && lidar_error_stats_ == 1)
    {
        log_warn("%s !!!!!!!!!!!!!!!!!!!! lidar error recovered this cycle !!!!!!!!!!!!!!!!!!!!!!!!!",__FUNCTION__);
        sendDeviceWarning(0);
        notifyDeviceWarningEvent("lidar_error", lidar_error_status, (uint64_t)now.toSec(), main_lidar_serial_num_);
        lidar_status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
        lidar_status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
        lidar_status_cmd_stru.event_data.data_item.event_status = lidar_error_status;
        lidar_status_cmd_stru.event_data.data_item.event_content = "lidar_error"; // check lidar erro event content field
        lidar_status_cmd_stru.event_data.data_item.event_serial_num = main_lidar_serial_num_; // deliberately empty
        diagEventCmdAdd(lidar_status_cmd_stru);
    }
    else
    {
        //log_info("no change to lidar error status lidar error status = %d", lidar_error_status);
    }

    lidar_error_stats_ = lidar_error_status;
    
}

void Diagnostics::checkUltraErrorEvent()
{
#if 0
    static int ultrasound_error_last_ = 0;
    static std::string ultrasound_serial_num_ = "";
    int ultrasound_error_this_ = -1;
    Json::Value root;
    char rand_str[16] = {0};
    // report ultrasound error
    ultrasound_error_this_ = diag_["robot_info"]["ultrasound"]["error"].asInt();

    if(ultrasound_error_this_ == 0 || ultrasound_error_this_ == 1)
    {
        if(1 == ultrasound_error_this_ && 0 == ultrasound_error_last_)
        {
            Utils::get_instance()->getRandStr(rand_str);
            ultrasound_serial_num_ = rand_str;
            //notifyRobotEvent(ULTRASONIC, ultrasound_error_this_, ultrasound_serial_num_);
        }
        else if(0 == ultrasound_error_this_ && 1 == ultrasound_error_last_)
        {
            //notifyRobotEvent(ULTRASONIC, ultrasound_error_this_, ultrasound_serial_num_);
        }
        else
        {

        }

        ultrasound_error_last_ = ultrasound_error_this_;
    }
#else
    static std::string ultrasound_serial_num_[4] = {""};
    static bool alarm_sent[4] = {false};
    tinyros::Time now = tinyros::Time::now();
    char rand_str[16] = {0};
    DataBaseCmdStru status_cmd_stru;

    
    for (int i = 0; i < 4; i++)
    {
        std::string event_content = "ultra_error_";

        int32_t data = diag_["robot_info"]["ultrasound"]["data"][i].asInt();

        event_content = event_content + std::to_string(i);

        log_info("%s --1-- , event_content:%s, ultraData[%d]:%d, alarm_sent:%d\r\n",__FUNCTION__, event_content.c_str(), i, data, alarm_sent[i]);

        if (data >= 1000  && !alarm_sent[i])
        {
            Utils::get_instance()->getRandStr(rand_str);
            ultrasound_serial_num_[i] = rand_str;

            log_info("%s --2-- , event_content:%s, ultraData[%d]:%d, alarm_sent:%d\r\n",__FUNCTION__, event_content.c_str(), i, data, alarm_sent[i]);

            notifyDeviceWarningEventNew(event_content, 1, (uint64_t)now.toSec(), ultrasound_serial_num_[i], "code", i);
            alarm_sent[i] = true;

            status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
            status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
            status_cmd_stru.event_data.data_item.event_status = 1;
            status_cmd_stru.event_data.data_item.event_content = event_content; // 
            status_cmd_stru.event_data.data_item.event_serial_num = ultrasound_serial_num_[i]; // deliberately empty
            // save it to database
            diagEventCmdAdd(status_cmd_stru);
        }
        else if (data < 1000 && alarm_sent[i])
        {
            log_info("%s --3-- , event_content:%s, ultraData[%d]:%d, alarm_sent:%d\r\n",__FUNCTION__, event_content.c_str(), i, data, alarm_sent[i]);

            notifyDeviceWarningEventNew(event_content, 0, (uint64_t)now.toSec(), ultrasound_serial_num_[i], "code", i);
            alarm_sent[i] = false;

            status_cmd_stru.cmd_type = EVENT_PROC_THIS_CYCLE;
            status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
            status_cmd_stru.event_data.data_item.event_status = 0;
            status_cmd_stru.event_data.data_item.event_content = event_content; // 
            status_cmd_stru.event_data.data_item.event_serial_num = ultrasound_serial_num_[i]; // deliberately empty
            // save it to database
            diagEventCmdAdd(status_cmd_stru);
        }
    }
#endif
}

void Diagnostics::checkGyroErrorEvent()
{
    static int gyro_error_last_ = 0;
    static std::string gyro_serial_num_ = "";
    int gyro_error_this_ = -1;
    Json::Value root;
    char rand_str[16] = {0};

    gyro_error_this_ = diag_["robot_info"]["gyro"]["error"].asInt();

    if(gyro_error_this_ == 0 || gyro_error_this_ == 1)
    {

        if(gyro_error_this_ ==1 && gyro_error_last_ == 0)
        {
            Utils::get_instance()->getRandStr(rand_str);
            gyro_serial_num_ = rand_str;
            //notifyRobotEvent(GYRO, gyro_error_this_, gyro_serial_num_);
        }
        else if(gyro_error_this_ ==0 && gyro_error_last_ == 1)
        {
            root["type"] = "default";
            root["value"] = "";
            //notifyRobotEvent(GYRO, gyro_error_this_, gyro_serial_num_);
        }
        else
        {

        }

        gyro_error_last_ = gyro_error_this_;

    }

}

// update gps receive time
void Diagnostics::updateGpsRecvTime(void)
{
    //log_info("%s recv gps from gps module",__FUNCTION__);
    struct timeval gps_recv_time_this;
    gettimeofday(&gps_recv_time_this, NULL);
    gps_recv_time_last_ = gps_recv_time_this.tv_sec;
}

// notify gps status
void Diagnostics::checkGpsStats()
{
    static int gps_error_last_ = -2;
    int gps_stats_this_;
    if (!diag_["robot_info"]["gps"]["error"].isNull())
    {
        gps_stats_this_ = diag_["robot_info"]["gps"]["error"].asInt();

        if(gps_stats_this_ == -1 || gps_stats_this_ == 0 || gps_stats_this_ == 1)
        {
            if(gps_stats_this_ != gps_error_last_)
            {
                notifyGpsStatus(gps_stats_this_);
            }

            gps_error_last_ = gps_stats_this_;
        }
    }
}

int Diagnostics::deterBatteryError(unsigned short low_byte, unsigned short mid_byte, unsigned short high_byte)
{
    int battery_error = 0;
    //log_info("%s low byte : %02x , mid byte : %02x , high byte : %02x",__FUNCTION__, low_byte , mid_byte , high_byte);
    int charge_temperature_high;
    int charge_temperature_low;
    int discharge_temperature_high;
    int discharge_temperature_low;
    
    charge_temperature_high = (low_byte >> 6) & 0x3;
    charge_temperature_low = (low_byte >> 8) & 0x3;
    discharge_temperature_high = (low_byte >> 10) & 0x3;
    discharge_temperature_low = (low_byte >> 12) & 0x3;

    //log_info("charge_temperature_high level : %d, charge_temperature_low level : %d, discharge_temperature_high level : %d, discharge_temperature_low level : %d", charge_temperature_high, charge_temperature_low, discharge_temperature_high, discharge_temperature_low);
    if(charge_temperature_high !=0 || charge_temperature_low !=0 || discharge_temperature_high !=0 || discharge_temperature_low !=0)
    {
        battery_error = -1;
    }

    int steady_state_charge_current_overload;
    int steady_state_discharge_current_overload;
    int power_board_overtemp;
    int balance_mos_overtemp;

    steady_state_charge_current_overload = (mid_byte >> 10) & 0x3;
    steady_state_discharge_current_overload = (mid_byte >> 12) & 0x3;
    power_board_overtemp = (mid_byte >> 14) & 0x3;
    balance_mos_overtemp = (mid_byte >> 16) & 0x3;

    //log_info("steady_state_charge_current_overload level : %d, steady_state_discharge_current_overload level : %d, power_board_overtemp level : %d, balance_mos_overtemp level : %d", steady_state_charge_current_overload, steady_state_discharge_current_overload, power_board_overtemp, balance_mos_overtemp);
    if(steady_state_charge_current_overload !=0 || steady_state_discharge_current_overload !=0 || power_board_overtemp !=0 || balance_mos_overtemp !=0)
    {
        battery_error = -1;
    }

    int transient_charge_current_overload;
    int transient_discharge_current_overload;
    int front_end_chip_error;
    int circuit_short_error;
    int charge_mos_error;
    int rs485_comm_error;

    transient_charge_current_overload = (high_byte) & 0x1;
    transient_discharge_current_overload = (high_byte >> 1) & 0x1;
    front_end_chip_error = (high_byte >> 2) & 0x1;
    circuit_short_error = (high_byte >> 3) & 0x1;
    charge_mos_error = (high_byte >> 4) & 0x1;
    rs485_comm_error = (high_byte >> 6) & 0x1;
    //log_info("transient_charge_current_overload level : %d, transient_discharge_current_overload level : %d, front_end_chip_error level : %d, circuit_short_error level : %d, charge_mos_error level : %d, rs485_comm_error level: %d", transient_charge_current_overload, transient_discharge_current_overload, front_end_chip_error, circuit_short_error, charge_mos_error, rs485_comm_error);
    if(transient_charge_current_overload !=0 || transient_discharge_current_overload !=0 || front_end_chip_error !=0 || circuit_short_error !=0 || charge_mos_error != 0 || rs485_comm_error != 0)
    {
        battery_error = -1;
    }

    return battery_error;
}

bool Diagnostics::doGetOtaPreCondition(atris_msgs::GetOTAPreCondition::Request& req, 
    atris_msgs::GetOTAPreCondition::Response& res) {
    //res.brake_status = getRobotBraked();
    res.battery_level = getBatLevel();
}

bool Diagnostics::doGetDiagnosticsInfo(atris_msgs::GetDiagnosticsInfo::Request& req,
    atris_msgs::GetDiagnosticsInfo::Response& res) {
    Json::FastWriter jwriter;
    ros::Time now = ros::Time::now();
    std::stringstream uid; uid << ((uint64_t) (now.toSec() * 1000000000ull));
    Json::Value content = diagnosticsRobotInfo(uid.str(), (uint64_t) (now.toSec() * 1000000000ull));
    res.data = jwriter.write(content);
}


#define SWUPGRADE_IMX_SUCCESS "success"
#define SWUPGRADE_IMX_FAILED "failed"
#define SWUPGRADE_IMX_STANDBY "standby"
#define SWUPGRADE_IMX_PROGRESS "progress"
std::string Diagnostics::getCurrentUpgradeStatus() {
    char buffer[1024];
    FILE *fp = NULL;
    char *s = NULL;
    std::string status = SWUPGRADE_IMX_STANDBY;

    memset(buffer, '\0', sizeof(buffer));
    if((fp = popen("sudo fw_printenv recovery_status", "r"))) {
        while(fgets(buffer, sizeof(buffer) - 1, fp)) {
            if((s = strstr(buffer, SWUPGRADE_IMX_SUCCESS))) {
                status = SWUPGRADE_IMX_SUCCESS;
                break;
            } else if ((s = strstr(buffer, SWUPGRADE_IMX_FAILED))) {
                status = SWUPGRADE_IMX_FAILED;
                break;
            } else if ((s = strstr(buffer, SWUPGRADE_IMX_STANDBY))) {
                status = SWUPGRADE_IMX_STANDBY;
                break;
            } else if ((s = strstr(buffer, SWUPGRADE_IMX_PROGRESS))) {
                status = SWUPGRADE_IMX_PROGRESS;
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

void Diagnostics::notifyBatteryWarningEvent(const std::string & event_content ,const uint64_t & timestamp, const std::string & serial_number)
{
    Json::Value content;
    Json::Value evt_arr;
    Json::Value battery_event_msg;
    std::string title;
    Json::FastWriter writer;
    std::string event_content_str = "";
    int iRet;

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string accid = std::string(shmrbt.robot.accid);

    content["category"] = "custom_event"; // custom event
    content["eventId"] = get_event_id_by_content(event_content);
    title = "notify_event_abnormal_warning";
    // accid
    content["userId"] = accid;

    // timestamp
    content["recordedAt"] = timestamp;
    // segmentaition is NULL
    content["segmentation"] = "";

    content["customSegmentation"]["serialNum"] = serial_number;
    content["customSegmentation"]["level"] = get_warn_level_by_content(event_content); // get event warn level

    // reason
    content["customSegmentation"]["reason"]["type"] = "code";
    content["customSegmentation"]["reason"]["value"] = 0;

    evt_arr.append(content);
    event_content_str = writer.write(evt_arr);
    log_info("%s event_content string : %s",__FUNCTION__, event_content_str.c_str());

    battery_event_msg["event_message"] = evt_arr; // put the event content into event message field to parse
    log_info("%s publish message out", __FUNCTION__);

    iRet = checkCanSend(title, content["eventId"].asString(), battery_event_msg);
    if(0 > iRet)
    {
        log_warn("%s not ready to send event msg, temporarily store it to database",__FUNCTION__);
    }
    else
    {
        log_info("%s ok to send robot event msg , post it to web...",__FUNCTION__);
    }
}

void Diagnostics::notifyAbnormalWarningEvent(const std::string & event_content , const int & status, const uint64_t & timestamp, const std::string & serial_number)
{
    Json::Value content;
    Json::Value evt_arr;
    Json::Value device_event_msg;
    std::string title;
    Json::FastWriter writer;
    std::string event_content_str = "";
    int iRet;

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string accid = std::string(shmrbt.robot.accid);

    content["category"] = "custom_event"; // custom event
    content["eventId"] = get_event_id_by_content(event_content);
    title = "notify_event_abnormal_warning";
    // accid
    content["userId"] = accid;

    // timestamp
    content["recordedAt"] = timestamp;
    // segmentaition is NULL
    content["segmentation"] = "";

    content["customSegmentation"]["status"] = status;

    content["customSegmentation"]["serialNum"] = serial_number;
    content["customSegmentation"]["level"] = get_warn_level_by_content(event_content); // get event warn level

    // reason
    content["customSegmentation"]["reason"]["type"] = "plain";
    content["customSegmentation"]["reason"]["value"] = "";

    evt_arr.append(content);
    event_content_str = writer.write(evt_arr);
    log_info("%s event_content string : %s",__FUNCTION__, event_content_str.c_str());

    device_event_msg["event_message"] = evt_arr; // put the event content into event message field to parse
    log_info("%s publish message out", __FUNCTION__);

    iRet = checkCanSend(title, content["eventId"].asString(), device_event_msg);
    if(0 > iRet)
    {
        log_warn("%s not ready to send event msg, temporarily store it to database",__FUNCTION__);
    }
    else
    {
        log_info("%s ok to send robot event msg , post it to web...",__FUNCTION__);
    }
}

// notify device warning status
void Diagnostics::notifyDeviceWarningEvent(const std::string & event_content , const int & status, const uint64_t & timestamp, const std::string & serial_number)
{
    Json::Value content;
    Json::Value evt_arr;
    Json::Value device_event_msg;
    std::string title;
    Json::FastWriter writer;
    std::string event_content_str = "";
    int iRet;

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string accid = std::string(shmrbt.robot.accid);

    content["category"] = "custom_event"; // custom event
    content["eventId"] = get_event_id_by_content(event_content);
    title = "notify_event_device_warning";
    // accid
    content["userId"] = accid;

    // timestamp
    content["recordedAt"] = timestamp;
    // segmentaition is NULL
    content["segmentation"] = "";

    content["customSegmentation"]["status"] = status;

    content["customSegmentation"]["serialNum"] = serial_number;
    content["customSegmentation"]["level"] = get_warn_level_by_content(event_content); // get event warn level

    int alarm_type = get_alarm_type_by_content(event_content);
    if (alarm_type != -1)
    {
        content["customSegmentation"]["reason"]["type"] = "code";
        content["customSegmentation"]["reason"]["value"] = alarm_type;
    }
    else
    {   // reason
        content["customSegmentation"]["reason"]["type"] = "plain";
        content["customSegmentation"]["reason"]["value"] = "";
    }

    evt_arr.append(content);
    event_content_str = writer.write(evt_arr);
    log_info("%s event_content string : %s",__FUNCTION__, event_content_str.c_str());

    device_event_msg["event_message"] = evt_arr; // put the event content into event message field to parse
    log_info("%s publish message out", __FUNCTION__);

    iRet = checkCanSend(title, content["eventId"].asString(), device_event_msg);
    if(0 > iRet)
    {
        log_warn("%s not ready to send event msg, temporarily store it to database",__FUNCTION__);
    }
    else
    {
        log_info("%s ok to send robot event message , post it to web",__FUNCTION__);
    }
}

void Diagnostics::notifyGpsStatus(int gps_status)
{
    Json::Value root;
    Json::Value js_evt;
    std::string event_content = "";
    Json::FastWriter writer;

    js_evt["event_type"] = "gps";

    js_evt["gps"] = gps_status;

    root["result"] = "success";
    root["result_code"] = 0;
    root["events"].append(js_evt);
    log_info("%s gps_status = %d",__FUNCTION__, gps_status);
    event_content = writer.write(root);
    log_warn("%s event_content is %s",__FUNCTION__, event_content.c_str());
    
    Utils::get_instance()->NotifyRobotStatus("notify_event",root,"");
}

void Diagnostics::notifyBatteryEvent(int bat_level)
{
    Json::Value root;
    Json::Value js_evt;
    std::string event_content = "";
    Json::FastWriter writer;

    js_evt["event_type"] = "battery";

    js_evt["battery"] = bat_level;

    root["result"] = "success";
    root["result_code"] = 0;
    root["events"].append(js_evt);
    log_info("---------------- %s battery level = %d ----------------------\r\n",__FUNCTION__, bat_level);
    event_content = writer.write(root);
    log_warn("%s event_content is %s",__FUNCTION__, event_content.c_str());
    
    Utils::get_instance()->NotifyRobotStatus("notify_event",root,"");
}

void Diagnostics::notifyUpgradeEvent(bool upgrade_stats)
{
    Json::Value root;
    Json::Value js_evt;
    std::string event_content = "";
    Json::FastWriter writer;

    js_evt["event_type"] = "upgrade";

    js_evt["upgrade"] = upgrade_stats?1:0;

    root["result"] = "success";
    root["result_code"] = 0;
    root["events"].append(js_evt);
    log_info("%s upgrade status = %d",__FUNCTION__, js_evt["upgrade"].asInt());
    event_content = writer.write(root);
    log_warn("%s event_content is %s",__FUNCTION__, event_content.c_str());
    
    Utils::get_instance()->NotifyRobotStatus("notify_event",root,"");
}

// return the ms value of current system time
long long Diagnostics::__getmstime(void)
{
    timeval tv;
    gettimeofday(&tv, NULL);
    return ((long long)tv.tv_sec) * 1000 + tv.tv_usec / 1000;
}
#if 0
void Diagnostics::postEventDelayThread(void)
{
    log_info("%s",__FUNCTION__);
    long long post_event_thread_called_time_this;
    long long post_event_delay_notify;
    post_event_thread_called_time_this = __getmstime();
    //log_info("%s thread start timestamp = %llu", __FUNCTION__, post_event_thread_called_time_this);
    boost::unique_lock<boost::mutex> lock(Diagnostics::event_list_mutex_, boost::defer_lock);
    while(1)
    {
        lock.lock();
        while(!event_list_.size()) 
        {
            log_warn("%s event list is currently empty",__FUNCTION__);
            event_list_cond_.wait(lock);
            post_event_delay_notify = __getmstime();
            log_warn("%s time debug , time difference = %llu", __FUNCTION__, post_event_delay_notify - post_event_thread_called_time_this);
        }

        postDelayedEvent();

        lock.unlock();
    }
}

void Diagnostics::postDelayedEvent(void)
{
    log_info("%s called\r\n",__FUNCTION__);
    std::list<PostToWebContent>::iterator iter = event_list_.begin();
    int i = 0;
    std::string event_content = "";
    Json::FastWriter writer;

    for(;iter != event_list_.end();iter++)
    {
        event_content = writer.write((*iter).get_event_content());
        log_info("%d th msg delayed post to web, title = %s, content = %s",i, (*iter).get_event_title().c_str(), event_content.c_str());
        Utils::get_instance()->NotifyRobotStatus((*iter).get_event_title(), (*iter).get_event_content());
        i++;
    }

    event_list_.clear();

    log_info("%s total num of event post to web = %d\r\n",__FUNCTION__, i);
}

int Diagnostics::postToWebDelay(void)
{
    {
        boost::unique_lock<boost::mutex> post_event_send_lock(Diagnostics::wait_can_send_mutex_);
        log_warn("%s set wait can send variable to true",__FUNCTION__);
        wait_can_send_ = true;
    }

    {
        boost::unique_lock<boost::mutex> lock(Diagnostics::event_list_mutex_);
        log_warn("%s notify event can send", __FUNCTION__);
        event_list_cond_.notify_all();
    }

    return 0;
}
#endif

// check if we can post the event message
// if not temporarily store it into database
int Diagnostics::checkCanSend(const std::string & title, const std::string & eventId, const Json::Value & js_str)
{
    //boost::unique_lock<boost::mutex> post_event_send_lock(wait_can_send_mutex_);
    Json::FastWriter writer;
    std::string event_content = "";

    event_content = writer.write(js_str);

    if(getConnectState())
    {
        // if network status ok , directly post it to web
        log_info("%s network status ok , directly post it to web",__FUNCTION__);
        Utils::get_instance()->NotifyRobotStatus(title, js_str, "");
        return 0;
    }
    else
    {
        // temp store to event store data base
        log_warn("%s network status not ok , temporarily store the warning event to event database, title = %s , event id = %s , event content = %s",__FUNCTION__, title.c_str(), eventId.c_str(), event_content.c_str());
        diagStoreEventAdd(eventId, js_str);
        return -1;
    }
}

// get main control board software version
std::string Diagnostics::getMainVersion(void)
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

void Diagnostics::on_recv_mcu_upload_log_status(const atris_msgs::McuLogCtrl& msg)
{
    if(msg.upload_log_msg == atris_msgs::McuLogCtrl::FINISH_UPLOAD_LOG)
    {
        if(msg.board_name == "chassis")
        {
            boost::unique_lock<boost::mutex> lock(Diagnostics::chassis_upload_log_mutex_);
            log_warn("%s notify chassis upload log finish", __FUNCTION__);
            chassis_upload_log_finish_ = true;
            chassis_upload_log_cv_.notify_all();
        }
        else
        {
            boost::unique_lock<boost::mutex> lock(Diagnostics::monitor_upload_log_mutex_);
            log_warn("%s notify monitor upload log finish", __FUNCTION__);
            monitor_upload_log_finish_ = true;
            monitor_upload_log_cv_.notify_all();
        }
    }
}

// create chassis and monitor log file folder if it does not exist
int Diagnostics::createMcuUploadDir(void)
{
    std::string monitor_upload_log_path;
    std::string chassis_upload_log_path;

    std::string monitor_log_position(MONITOR_LOG_FILE_FOLDER_PATH);
    std::string chassis_log_position(CHASSIS_LOG_FILE_FOLDER_PATH);

    chassis_upload_log_path = "/userdata/tmp" + chassis_log_position;
    monitor_upload_log_path = "/userdata/tmp" + monitor_log_position;
    log_info("%s chassis upload log path = %s",__FUNCTION__, chassis_upload_log_path.c_str());
    log_info("%s monitor upload log path = %s",__FUNCTION__, monitor_upload_log_path.c_str());

    if(!createMultiLevelDir(chassis_upload_log_path.c_str()))
    {
        log_error("%s create folder for chassis to upload log files failed!!!",__FUNCTION__);
        return -1;
    }
    else
    {
        log_info("%s create chassis log folder success...",__FUNCTION__);
    }

    if(!createMultiLevelDir(monitor_upload_log_path.c_str()))
    {
        log_error("%s create folder for monitor to upload log files failed!!!",__FUNCTION__);
        return -1;
    }
    else
    {
        log_info("%s create monitor log folder success...",__FUNCTION__);
    }

    return 0;
}

bool Diagnostics::createMultiLevelDir(const char *path)
{
    int i, len;
    char dir_path[256] = {0};
    len = strlen(path);
    dir_path[len] = '\0';
    strncpy(dir_path, path, len);
    log_info("%s directory path : %s",__FUNCTION__, path);
    log_info("%s len = %d",__FUNCTION__, len);
    for (i=0; i<len; i++)
    {
        if (dir_path[i] == '/' && i > 0)
        {
            dir_path[i]='\0';
            log_info("dir path = %s",dir_path);
            if (access(dir_path, F_OK) < 0)
            {
                if (mkdir(dir_path, 0777) < 0)
                {
                    log_error("mkdir=%s:msg=%s\n", dir_path, strerror(errno));
                    return false;
                }
                else
                {
                    log_info("make dir success , dir path : %s\r\n",dir_path);
                    //return true;
                }
            }
            dir_path[i]='/';
        }
    }

    return true;
}

void Diagnostics::sendMcuUploadLogReq(const std::string & board_name)
{
    atris_msgs::McuLogCtrl mcu_log_req;

    mcu_log_req.board_name = board_name;
    mcu_log_req.upload_log_msg = atris_msgs::McuLogCtrl::START_UPLOAD_LOG;

    mcu_upload_log_req_pub_.publish(mcu_log_req);
}

int Diagnostics::waitChassisUploadLogFinish(void)
{
    int count = 0;
    int nmax_count = 60;
    
    log_warn("%s wait chassis upload log finish start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n",__FUNCTION__);

    boost::unique_lock<boost::mutex> check_upload_lock(chassis_upload_log_mutex_, boost::defer_lock);

    while(count < nmax_count)
    {
        check_upload_lock.lock();
        if(chassis_upload_log_finish_)
        {
            check_upload_lock.unlock();
            break;
        }

        chassis_upload_log_cv_.timed_wait(check_upload_lock, boost::get_system_time() + boost::posix_time::milliseconds(1000*1));
        if(chassis_upload_log_finish_)
        {
            check_upload_lock.unlock();
            break;
        }

        count++;

        check_upload_lock.unlock();

        log_info("%s check chassis upload logs wait , count: %d",__FUNCTION__, count);
    }

    if(count == nmax_count)
    {
        log_error("%s chassis upload timeout!!!\r\n",__FUNCTION__);
        return -1;
    }
    else
    {
        log_info("%s chassis upload log finish...\r\n",__FUNCTION__);
    }

    return 0;
}

int Diagnostics::waitMonitorUploadLogFinish(void)
{
    int count = 0;
    int nmax_count = 60;
    
    log_warn("%s wait monitor upload log finish start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n",__FUNCTION__);

    boost::unique_lock<boost::mutex> check_upload_lock(monitor_upload_log_mutex_, boost::defer_lock);

    while(count < nmax_count)
    {
        check_upload_lock.lock();
        if(monitor_upload_log_finish_)
        {
            check_upload_lock.unlock();
            break;
        }

        monitor_upload_log_cv_.timed_wait(check_upload_lock, boost::get_system_time() + boost::posix_time::milliseconds(1000*1));
        if(monitor_upload_log_finish_)
        {
            check_upload_lock.unlock();
            break;
        }

        count++;

        check_upload_lock.unlock();

        log_info("%s check monitor upload logs wait , count: %d",__FUNCTION__, count);
    }

    if(count == nmax_count)
    {
        log_error("%s monitor upload timeout!!!\r\n",__FUNCTION__);
        return -1;
    }
    else
    {
        log_info("%s monitor upload log finish...\r\n",__FUNCTION__);
    }

    return 0;
}

int Diagnostics::reqChassisLogs(void)
{
    int iRet;
    std::string board_name = "chassis";
    chassis_upload_log_finish_ = false;
    sendMcuUploadLogReq(board_name);
    iRet = waitChassisUploadLogFinish();
    if(iRet < 0)
    {
        log_error("%s wait chassis upload log failed!!! iRet = %d",__FUNCTION__, iRet);
        return -1;
    }
    else
    {
        log_info("%s wait chassis upload log success...\r\n", __FUNCTION__);
    }

    return 0;
}

int Diagnostics::reqMonitorLogs(void)
{
    int iRet;
    std::string board_name = "monitor";
    monitor_upload_log_finish_ = false;
    sendMcuUploadLogReq(board_name);
    iRet = waitMonitorUploadLogFinish();
    if(iRet < 0)
    {
        log_error("%s wait monitor upload log failed!!! iRet = %d",__FUNCTION__, iRet);
        return -1;
    }
    else
    {
        log_info("%s wait monitor upload log success...\r\n", __FUNCTION__);
    }

    return 0;
}

// get system up time
int Diagnostics::getSysUpTime(float & running_hours)
{
    struct sysinfo info;
    //char run_time[128];
 
    if (sysinfo(&info)) {
        fprintf(stderr, "Failed to get sysinfo, errno:%u, reason:%s\n",errno, strerror(errno));
        return -1;
    }
 
    long timenum=info.uptime;
    int runday=timenum/86400;
    int runhour=(timenum%86400)/3600;
    int runmin=(timenum%3600)/60;
    int runsec=timenum%60;
    //bzero(run_time, 128);

    running_hours = runhour + runmin/60.0 + runhour/3600.0;

    return 0;
}

#if 0
void Diagnostics::getOdomInfo(Json::Value & tmp)
{
    float sum_spd;
    sum_spd = sqrt(pow(odom_topic_info.speed_x,2) + pow(odom_topic_info.speed_y,2));
    tmp["robot_info"]["odom"]["error"] = odom_error_;
    tmp["robot_info"]["odom"]["odo"] = odom_topic_info.dist;
    tmp["robot_info"]["odom"]["speed_linear"] = sum_spd;
    tmp["robot_info"]["odom"]["speed_theta"] = odom_topic_info.speed_a;
    log_info("%s odo : %lf , speed linear %lf , speed theta : %lf",__FUNCTION__, odom_topic_info.dist, sum_spd, odom_topic_info.speed_a);
}
#endif

void Diagnostics::checkGpsError(void)
{
    struct timeval gps_check_this;
    gettimeofday(&gps_check_this,NULL);

    int gps_error_this;

    if((gps_check_this.tv_sec - gps_recv_time_last_) > 5)
    {
        gps_error_this = -1;
    }

    {
        boost::unique_lock<boost::mutex> lock(mutex_);
        diag_["robot_info"]["gps"]["error"] = gps_error_this;
    }

}

void Diagnostics::checkImuError(void)
{
    struct timeval imu_check_this;
    gettimeofday(&imu_check_this,NULL);

    //static long long imu_update_last_;

    int imu_error_this;

    if((imu_check_this.tv_sec - imu_recv_time_last_) > 5)
    {
        imu_error_this = -1;
    }
    else
    {
        imu_error_this = 0;
    }

    if(imu_error_ !=  imu_error_this)
    {
        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            diag_["robot_info"]["imu"]["error"] = imu_error_this;
        }
    }

    imu_error_ = imu_error_this;
}

// just ping the lidar ip
void Diagnostics::checkDevStatus(void)
{

    while (1)
    {
        checkLidarDev();

        checkBmsDev();

        checkChassisDriver();

        checkPtzDev();

        checkCCBBoard();

        checkSYSBoard();

        checkImuError();

        checkGpsError();

        UpDateOdomInfo();

        usleep(1000*1000); // check device every 1 second to ping the device
    }
}

void Diagnostics::UpDateOdomInfo(void)
{

    struct timeval odom_check_this;
    gettimeofday(&odom_check_this,NULL);

    static long long odom_update_last_;

    int odom_error_this;

    if((odom_check_this.tv_sec - odom_recv_time_last_) > 5)
    {
        odom_error_this = 1;
    }
    else
    {
        odom_error_this = 0;
    }

    if(odom_error_ !=  odom_error_this || ((odom_check_this.tv_sec - odom_update_last_) > 5))
    {
        // update the data
        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            float sum_spd;
            sum_spd = sqrt(pow(odom_topic_info.speed_x,2) + pow(odom_topic_info.speed_y,2));
            diag_["robot_info"]["odom"]["error"] = odom_error_this;
            diag_["robot_info"]["odom"]["odo"] = doubleTrim(odom_topic_info.dist);
            diag_["robot_info"]["odom"]["speed_linear"] = doubleTrim(sum_spd);
            diag_["robot_info"]["odom"]["speed_theta"] = doubleTrim(odom_topic_info.speed_a);

        }

        odom_update_last_ = odom_check_this.tv_sec;
    }

    odom_error_ = odom_error_this;
}

void Diagnostics::checkBmsDev(void)
{
    struct timeval bms_check_this;
    gettimeofday(&bms_check_this,NULL);

    int bms_recv_timeout = 0;
    int bms_error_status = 0;
    if(simulation_event_switch_)
    {
        log_info("%s we are in event simulaiton mode, skip checking real bms data",__FUNCTION__);
        return;
    }

    if((bms_info_recv_last_ != -1) && ((bms_check_this.tv_sec - bms_info_recv_last_) > 5))
    {
        log_warn("%s bms recv timeout , current sec = %lld, bms info recv last = %lld , diff = %lld\r\n", __FUNCTION__, bms_check_this.tv_sec, bms_info_recv_last_, bms_check_this.tv_sec - bms_info_recv_last_);
        bms_recv_timeout = 1;
        bms_communication_ok_ = 0;
    }
    else
    {
        log_info("%s bms not timeout...",__FUNCTION__);
        bms_recv_timeout = 0;
    }

    if(bms_recv_timeout || bms_warning_stats_ == 1)
    {
        bms_error_status = 1;
    }
    else
    {
        bms_error_status = 0;
    }

    log_info("%s bms error status : %d\r\n",__FUNCTION__, bms_error_status);

    checkBMSWhenStartUp(bms_error_status);
    checkBMSErrorEvent(bms_error_status);
    
}

void Diagnostics::checkChassisDriver(void)
{
    struct timeval chassis_check_this;
    gettimeofday(&chassis_check_this,NULL);
    int chassis_error_status = 0;
    int chassis_recv_timeout = 0;

    if(simulation_event_switch_)
    {
        log_info("%s we are in event simulaiton mode, skip checking real bms data",__FUNCTION__);
        return;
    }
    
    // not checking chassis error in following conditions
    if(bms_communication_ok_ != 1 || (bms_communication_ok_ && isBmsCharging_) || (machine_status_ == 1))
    {
        return;
    }

    if(chassis_lf_error_stats_==-1||chassis_rf_error_stats_==-1||chassis_lr_error_stats_==-1||chassis_rr_error_stats_==-1)
    {
        log_error("%s chassis four wheel error info collected not completed",__FUNCTION__);
        return;
    }

    if((chassis_lf_recv_last_!=-1 && chassis_lr_recv_last_!=-1 && chassis_rf_recv_last_!=-1 && chassis_rr_recv_last_!=-1) && 
        (((chassis_check_this.tv_sec - chassis_lf_recv_last_) > 10) || ((chassis_check_this.tv_sec - chassis_lr_recv_last_) > 10) || ((chassis_check_this.tv_sec - chassis_rf_recv_last_) > 10) || ((chassis_check_this.tv_sec - chassis_rr_recv_last_) > 10)))
    {
        log_warn("%s chassis recv timeout , current sec = %lld, chassis lf recv last = %lld , diff = %lld\r\n", __FUNCTION__, chassis_check_this.tv_sec, chassis_lf_recv_last_, chassis_check_this.tv_sec - chassis_lf_recv_last_);
        log_warn("%s chassis recv timeout , current sec = %lld, chassis lr recv last = %lld , diff = %lld\r\n", __FUNCTION__, chassis_check_this.tv_sec, chassis_lr_recv_last_, chassis_check_this.tv_sec - chassis_lr_recv_last_);
        log_warn("%s chassis recv timeout , current sec = %lld, chassis rf recv last = %lld , diff = %lld\r\n", __FUNCTION__, chassis_check_this.tv_sec, chassis_rf_recv_last_, chassis_check_this.tv_sec - chassis_rf_recv_last_);
        log_warn("%s chassis recv timeout , current sec = %lld, chassis rr recv last = %lld , diff = %lld\r\n", __FUNCTION__, chassis_check_this.tv_sec, chassis_rr_recv_last_, chassis_check_this.tv_sec - chassis_rr_recv_last_);

        chassis_recv_timeout = 1;
    }
    else
    {
        log_info("%s chassis not timeout...",__FUNCTION__);
        chassis_recv_timeout = 0;
    }

    if(chassis_recv_timeout || chassis_warning_stats_ == 1)
    {
        chassis_error_status = 1;
    }
    else
    {
        chassis_error_status = 0;
    }

    log_info("chassis driver error status = %d\r\n",chassis_error_status);

    checkChassisWhenStartUp(chassis_error_status);
    checkChassisErrorEvent(chassis_error_status);
}

void Diagnostics::checkPtzDev(void)
{
    const char * ptz_ip = PTZ_IP;
    static int ptz_acc_cnt = 0;
    static int ptz_recov_cnt = 0;
    bool ptz_state_stable = false;
    int ptz_network_status;

    if(simulation_event_switch_)
    {
        log_info("%s we are in event simulaiton mode, skip checking real bms data",__FUNCTION__);
        return;
    }
    
    if(!system_delay_check_)
    {
        log_info("%s system delay check not ok",__FUNCTION__);
        ptz_acc_cnt = 0;
        ptz_recov_cnt = 0;
        return;
    }

    if(bms_communication_ok_ != 1 || (bms_communication_ok_ == 1 && isBmsCharging_) || (machine_status_ == 1))
    {
        log_info("%s bms communication ok : %d, bms is charging : %d or machine in standby mode : %d", __FUNCTION__, bms_communication_ok_, isBmsCharging_?1:0, machine_status_);
        ptz_acc_cnt = 0;
        ptz_recov_cnt = 0;
        return;
    }

    if (!utils->check_network_state(ptz_ip))
    {
        ptz_network_status = 1;
        log_error("\033[1;31mptz network unreachable.%s\033[0m", ptz_ip);

    }
    else
    {
        ptz_network_status = 0;

    }

    if(ptz_network_status == 1)
    {
        if(ptz_acc_cnt != PTZ_ERROR_CNT)
        {
            ptz_acc_cnt++;
        }

        if(ptz_recov_cnt != 0)
        {
            ptz_recov_cnt--;
        }
    }
    else if(ptz_network_status == 0)
    {
        if(ptz_recov_cnt != PTZ_RECOVER_CNT)
        {
            ptz_recov_cnt++;
        }

        if(ptz_acc_cnt != 0)
        {
            ptz_acc_cnt--;
        }
    }
    else
    {
        log_error("%s undetermined ptz error status : %d",__FUNCTION__, ptz_network_status);
    }

    if(ptz_acc_cnt == PTZ_ERROR_CNT || ptz_recov_cnt == PTZ_RECOVER_CNT)
    {
        //log_info("%s ptz network state stable , ptz connect status : %d",__FUNCTION__, ptz_network_status);

        ptz_state_stable = true;
    }

    if(ptz_state_stable)
    {
        checkPTZWhenStartUp(ptz_network_status);
        checkPTZErrorEvent(ptz_network_status);
    }

}

void Diagnostics::checkCCBBoard(void)
{
    const char * ccb_ip = CCB_IP;
    static int ccb_acc_cnt = 0;
    static int ccb_recov_cnt = 0;
    bool ccb_state_stable = false;
    int ccb_network_status;

    if(simulation_event_switch_)
    {
        log_info("%s we are in event simulaiton mode, skip checking real bms data",__FUNCTION__);
        return;
    }

    if (!utils->check_network_state(ccb_ip))
    {
        ccb_network_status = 1;
        log_error("\033[1;31mccb network unreachable.%s\033[0m", ccb_ip);

    }
    else
    {
        ccb_network_status = 0;

    }

    if(ccb_network_status == 1)
    {
        if(ccb_acc_cnt != CCB_ERROR_CNT)
        {
            ccb_acc_cnt++;
        }

        if(ccb_recov_cnt != 0)
        {
            ccb_recov_cnt--;
        }
    }
    else if(ccb_network_status == 0)
    {
        if(ccb_recov_cnt != CCB_RECOVER_CNT)
        {
            ccb_recov_cnt++;
        }

        if(ccb_acc_cnt != 0)
        {
            ccb_acc_cnt--;
        }
    }
    else
    {
        log_error("%s undetermined ccb error status : %d",__FUNCTION__, ccb_network_status);
    }

    if(ccb_acc_cnt == CCB_ERROR_CNT || ccb_recov_cnt == CCB_RECOVER_CNT)
    {
        ccb_state_stable = true;
    }

    if(ccb_state_stable)
    {
        checkCCBWhenStartUp(ccb_network_status);
        checkCCBErrorEvent(ccb_network_status);
    }

}

void Diagnostics::checkSYSBoard(void)
{
    const char * sys_ip = SYS_IP;
    static int sys_acc_cnt = 0;
    static int sys_recov_cnt = 0;
    bool sys_state_stable = false;
    int sys_network_status;
    
    if(simulation_event_switch_)
    {
        log_info("%s we are in event simulaiton mode, skip checking real bms data",__FUNCTION__);
        return;
    }

    if (!utils->check_network_state(sys_ip))
    {
        sys_network_status = 1;
        log_error("\033[1;31msys network unreachable.%s\033[0m", sys_ip);

    }
    else
    {
        sys_network_status = 0;

    }

    if(sys_network_status == 1)
    {
        if(sys_acc_cnt != SYS_ERROR_CNT)
        {
            sys_acc_cnt++;
        }

        if(sys_recov_cnt != 0)
        {
            sys_recov_cnt--;
        }
    }
    else if(sys_network_status == 0)
    {
        if(sys_recov_cnt != SYS_RECOVER_CNT)
        {
            sys_recov_cnt++;
        }

        if(sys_acc_cnt != 0)
        {
            sys_acc_cnt--;
        }
    }
    else
    {
        log_error("%s undetermined sys error status : %d",__FUNCTION__, sys_network_status);
    }

    if(sys_acc_cnt == SYS_ERROR_CNT || sys_recov_cnt == SYS_RECOVER_CNT)
    {
        sys_state_stable = true;
    }

    if(sys_state_stable)
    {
        checkSYSWhenStartUp(sys_network_status);
        checkSYSErrorEvent(sys_network_status);
    }

}

void Diagnostics::checkLidarDev(void)
{
    const char * lidar_ip = LIDAR_IP;
    static int lidar_acc_cnt = 0;
    static int lidar_recov_cnt = 0;
    bool lidar_state_stable = false;
    int lidar_network_status;

    if(!system_delay_check_)
    {
        lidar_acc_cnt = 0;
        lidar_recov_cnt = 0;
        return;
    }

    if(bms_communication_ok_!=1 || (bms_communication_ok_ == 1 && isBmsCharging_) || (machine_status_ == 1))
    {
        lidar_acc_cnt = 0;
        lidar_recov_cnt = 0;
        return;
    }

    if (!utils->check_network_state(lidar_ip))
    {
        lidar_network_status = 1;
        log_error("\033[1;31mlidar network unreachable.%s\033[0m", lidar_ip);

    }
    else
    {
        lidar_network_status = 0;

    }

    if(lidar_network_status == 1)
    {
        if(lidar_acc_cnt != LIDAR_ERROR_CNT)
        {
            lidar_acc_cnt++;
        }

        if(lidar_recov_cnt != 0)
        {
            lidar_recov_cnt--;
        }
    }
    else if(lidar_network_status == 0)
    {
        if(lidar_recov_cnt != LIDAR_RECOVER_CNT)
        {
            lidar_recov_cnt++;
        }

        if(lidar_acc_cnt != 0)
        {
            lidar_acc_cnt--;
        }
    }
    else
    {
        log_error("%s undetermined lidar error status : %d",__FUNCTION__, lidar_network_status);
    }

    if(lidar_acc_cnt == LIDAR_ERROR_CNT || lidar_recov_cnt == LIDAR_RECOVER_CNT)
    {
        //log_info("%s lidar network state stable , lidar connect status : %d",__FUNCTION__, lidar_network_status);
        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            diag_["robot_info"]["main_lidar"]["error"] = lidar_network_status == 1?-1:0;
        }

        lidar_state_stable = true;
    }
    // report abnormal event
    if(lidar_state_stable)
    {
        checkLidarWhenStartUp(lidar_network_status);
    
        checkLidarErrorEvent(lidar_network_status);
    }
}

// recv diag info from chassis
void Diagnostics::on_recv_chassis_diag_info(const tinyros::atris_msgs::chassis_abnor_info &msg)
{
    struct timeval chassis_abnor_info_recv_this;
    gettimeofday(&chassis_abnor_info_recv_this,NULL);
    bool need_refresh = false;

    #if 1
    //log_info("%s time stamp : %ld",__FUNCTION__, msg.timestamp);
    //log_info("%s chassis abnor : %d",__FUNCTION__, msg.chassis_abnor);
    //log_info("%s chassis axle : %s",__FUNCTION__, msg.axle);
    //log_info("%s chassis steer_error_code : %d", __FUNCTION__, msg.steer_error_code);
    //log_info("%s chassis direct_error_code : %d", __FUNCTION__, msg.direct_error_code);
    //log_info("%s chassis steer 485 angle : %f",__FUNCTION__, msg.steer_485_angle);
    //log_info("%s chassis steer angle : %f",__FUNCTION__, msg.steer_angle);
    //log_info("%s chassis direct speed : %f",__FUNCTION__, msg.direct_speed);
    //log_info("%s chassis steer temperature : %d",__FUNCTION__, msg.steer_temperature);
    //log_info("%s chassis direct temperature : %d",__FUNCTION__, msg.direct_temperature);
    //log_info("%s chassis steer torque : %d",__FUNCTION__, msg.steer_torque);
    //log_info("%s chassis direct torque : %d",__FUNCTION__, msg.direct_torque);
    #endif

    checkDriverWarningEventStartup(msg);

    if(!strcmp(msg.axle,"leftFront"))
    {
        if(msg.chassis_abnor != chassis_lf_error_stats_ || ((chassis_abnor_info_recv_this.tv_sec - chassis_lf_recv_last_) > 5))
        {
            need_refresh = true;

            {
                boost::unique_lock<boost::mutex> lock(mutex_);
                diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["chassis_abnor"] = msg.chassis_abnor;
                diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["steer_error_code"] = msg.steer_error_code;
                diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["direct_error_code"] = msg.direct_error_code;
                diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["steer_485_angle"] = msg.steer_485_angle;
                diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["steer_angle"] = msg.steer_angle;
                diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["direct_speed"] = msg.direct_speed;
                diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["steer_temperature"] = msg.steer_temperature;
                diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["direct_temparature"] = msg.direct_temperature;
                diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["steer_torque"] = msg.steer_torque;
                diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["direct_torque"] = msg.direct_torque;
            }

            chassis_lf_error_stats_ = msg.chassis_abnor;
            chassis_lf_recv_last_ = chassis_abnor_info_recv_this.tv_sec;

            checkDriverLeftFrontWarningEvent(msg);
        }
    }
    else if(!strcmp(msg.axle,"rightFront"))
    {
        if(msg.chassis_abnor != chassis_rf_error_stats_ || ((chassis_abnor_info_recv_this.tv_sec - chassis_rf_recv_last_) > 5))
        {
            need_refresh = true;

            {
                boost::unique_lock<boost::mutex> lock(mutex_);
                diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["chassis_abnor"] = msg.chassis_abnor;
                diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["steer_error_code"] = msg.steer_error_code;
                diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["direct_error_code"] = msg.direct_error_code;
                diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["steer_485_angle"] = msg.steer_485_angle;
                diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["steer_angle"] = msg.steer_angle;
                diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["direct_speed"] = msg.direct_speed;
                diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["steer_temperature"] = msg.steer_temperature;
                diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["direct_temparature"] = msg.direct_temperature;
                diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["steer_torque"] = msg.steer_torque;
                diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["direct_torque"] = msg.direct_torque;
            }

            chassis_rf_error_stats_ = msg.chassis_abnor;
            chassis_rf_recv_last_ = chassis_abnor_info_recv_this.tv_sec;

            checkDriverRightFrontWarningEvent(msg);
        }
    }
    else if(!strcmp(msg.axle,"leftRear"))
    {
        if(msg.chassis_abnor != chassis_lr_error_stats_ || ((chassis_abnor_info_recv_this.tv_sec - chassis_lr_recv_last_) > 5))
        {
            need_refresh = true;

            {
                boost::unique_lock<boost::mutex> lock(mutex_);
                diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["chassis_abnor"] = msg.chassis_abnor;
                diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["steer_error_code"] = msg.steer_error_code;
                diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["direct_error_code"] = msg.direct_error_code;
                diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["steer_485_angle"] = msg.steer_485_angle;
                diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["steer_angle"] = msg.steer_angle;
                diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["direct_speed"] = msg.direct_speed;
                diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["steer_temperature"] = msg.steer_temperature;
                diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["direct_temparature"] = msg.direct_temperature;
                diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["steer_torque"] = msg.steer_torque;
                diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["direct_torque"] = msg.direct_torque;
            }

            chassis_lr_error_stats_ = msg.chassis_abnor;
            chassis_lr_recv_last_ = chassis_abnor_info_recv_this.tv_sec;

            checkDriverLeftRearWarningEvent(msg);
        }
    }
    else if(!strcmp(msg.axle,"rightRear"))
    {
        if(msg.chassis_abnor != chassis_rr_error_stats_ || ((chassis_abnor_info_recv_this.tv_sec - chassis_rr_recv_last_) > 5))
        {
            need_refresh = true;

            {
                boost::unique_lock<boost::mutex> lock(mutex_);
                diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["chassis_abnor"] = msg.chassis_abnor;
                diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["steer_error_code"] = msg.steer_error_code;
                diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["direct_error_code"] = msg.direct_error_code;
                diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["steer_485_angle"] = msg.steer_485_angle;
                diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["steer_angle"] = msg.steer_angle;
                diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["direct_speed"] = msg.direct_speed;
                diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["steer_temperature"] = msg.steer_temperature;
                diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["direct_temparature"] = msg.direct_temperature;
                diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["steer_torque"] = msg.steer_torque;
                diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["direct_torque"] = msg.direct_torque;
            }

            chassis_rr_error_stats_ = msg.chassis_abnor;
            chassis_rr_recv_last_ = chassis_abnor_info_recv_this.tv_sec;

            checkDriverRightRearWarningEvent(msg);
        }
    }
    else
    {
        //log_error("%s unrecognized axle name",__FUNCTION__);
    }

    #if 1
    if(need_refresh)
    {
        //log_info("&&&&&&&& %s need to refresh chassis diag info &&&&&&&&&&&&",__FUNCTION__);
        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            if(diag_["robot_info"]["chassis_driver"]["power"]["right_rear"]["chassis_abnor"].asInt()!=0 
                || diag_["robot_info"]["chassis_driver"]["power"]["left_rear"]["chassis_abnor"].asInt()!=0 
                || diag_["robot_info"]["chassis_driver"]["power"]["left_front"]["chassis_abnor"].asInt()!=0 
                || diag_["robot_info"]["chassis_driver"]["power"]["right_front"]["chassis_abnor"].asInt()!=0)
            {
                //log_info("%s set chassis driver error to -1", __FUNCTION__);
                diag_["robot_info"]["chassis_driver"]["error"] = -1;
                chassis_warning_stats_ = 1;
            }
            else
            {
                //log_info("%s set chassis driver error to 0", __FUNCTION__);
                diag_["robot_info"]["chassis_driver"]["error"] = 0;
                chassis_warning_stats_ = 0;
            }
        }
    }
    else
    {
        //log_info("%s do not need to refresh",__FUNCTION__);
    }

    #endif

    //checkChassisWhenStartUp();
    //checkChassisErrorEvent();


}

void Diagnostics::run_mode_thread(void)
{
    int current_run_mode;
    while(!mode_priority_thread_exit_)
    {
        sys_tick_ += 100;
        current_run_mode = determineCurrentRunMode();
        //log_info("%s current run mode is : %d",__FUNCTION__, current_run_mode);
        notifyCurrentRunMode(current_run_mode);
        usleep(1000 *100); // sleep 100ms
    }
}

int Diagnostics::getRunModeTransCode(int & run_mode)
{
    int proto_run_mode_code;
    if(run_mode == REMOTE_CONTROL_MODE_INDEX)
    {
        proto_run_mode_code = REMOTE_CONTROL_MODE;
    }
    else if(run_mode == LOW_BAT_RETRUN_MODE_INDEX)
    {
        proto_run_mode_code = LOW_BAT_RETRUN_MODE;
    }
    else if(run_mode == BACKGROUND_CONTROL_MODE_INDEX)
    {
        proto_run_mode_code = BACKGROUND_CONTROL_MODE;
    }
    else if(run_mode == EMERGENCY_TASK_MODE_INDEX)
    {
        proto_run_mode_code = EMERGENCY_TASK_MODE;
    }
    else if(run_mode == TASK_MODE_INDEX)
    {
        proto_run_mode_code = TASK_MODE;
    }
    else if(run_mode == IDLE_RETURN_MODE_INDEX)
    {
        proto_run_mode_code = IDLE_RETURN_MODE;
    }
    else if(run_mode == IDLE_MODE_INDEX)
    {
        proto_run_mode_code = IDLE_MODE;
    }

    return proto_run_mode_code;
}

std::string Diagnostics::getRunModeStr(int mode)
{
    std::string mode_str = "";
    switch(mode)
    {
        case REMOTE_CONTROL_MODE_INDEX:
            mode_str = "remote control mode";
        break;
        case LOW_BAT_RETRUN_MODE_INDEX:
            mode_str = "low bat return mode";
        break;
        case BACKGROUND_CONTROL_MODE_INDEX:
            mode_str = "background control mode";
        break;
        case EMERGENCY_TASK_MODE_INDEX:
            mode_str = "emergency task mode";
        break;
        case TASK_MODE_INDEX:
            mode_str = "task mode";
        break;
        case IDLE_RETURN_MODE_INDEX:
            mode_str = "idle return mode";
        break;
        case IDLE_MODE_INDEX:
            mode_str = "idle mode";
        break;
        default:
            log_error("%s unrecognized mode : %d", __FUNCTION__, mode);
        break;
    }

    return mode_str;
}

int Diagnostics::determineCurrentRunMode(void)
{
    int run_mode = IDLE_MODE_INDEX;
    int run_mode_code;

    boost::unique_lock<boost::mutex> mode_lock(run_mode_mutex_);

    for(int i = 0; i < NUMBER_OF_MODE_PRIORITY; i++)
    {
        if(run_mode_status_[i] == 1)
        {
            run_mode = i;
            //log_warn("%s robot run mode is set to %s",__FUNCTION__, getRunModeStr(i).c_str());
            break;
        }
    }

    run_mode_code = getRunModeTransCode(run_mode);

    return run_mode_code;

}

void Diagnostics::notifyCurrentRunMode(const int & current_robot_mode)
{
    if(current_robot_mode != robot_run_mode_)
    {
        // notify run_mode change to shadow
        log_warn("%s robot run mode changed : %d , notify it to shadow", __FUNCTION__, current_robot_mode);

        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            diag_["robot_info"]["base"]["run_mode"]  = current_robot_mode;
        }

        notifyRobotInfoToShadow();
    }
    
    robot_run_mode_ = current_robot_mode;
}

void Diagnostics::robotRunModeInfoCb(const atris_msgs::RobotRunMode &msg)
{
    log_warn("************************** %s robot mode : %d , status : %d",__FUNCTION__, msg.robot_mode, msg.status);
    setRobotRunMode(msg.robot_mode, msg.status);
}

void Diagnostics::setRobotRunMode(uint8_t robot_mode, uint8_t status)
{
    boost::unique_lock<boost::mutex> mode_lock(run_mode_mutex_);
    if(robot_mode == IDLE_MODE)
    {
        log_info("%s robot mode set to idle mode",__FUNCTION__);
        run_mode_status_[IDLE_MODE_INDEX] = status;
    }
    else if(robot_mode == TASK_MODE)
    {
        log_info("%s robot mode set to task mode",__FUNCTION__);
        run_mode_status_[TASK_MODE_INDEX] = status;
    }
    else if(robot_mode == EMERGENCY_TASK_MODE)
    {
        log_info("%s robot mode set to emergency task mode",__FUNCTION__);
        run_mode_status_[EMERGENCY_TASK_MODE_INDEX] = status;
    }
    else if(robot_mode == BACKGROUND_CONTROL_MODE)
    {
        log_info("%s robot mode set to background control mode",__FUNCTION__);
        run_mode_status_[BACKGROUND_CONTROL_MODE_INDEX] = status;
    }
    else if(robot_mode == REMOTE_CONTROL_MODE)
    {
        log_info("%s robot mode set to remote control mode",__FUNCTION__);
        run_mode_status_[REMOTE_CONTROL_MODE_INDEX] = status;
    }
    else if(robot_mode == LOW_BAT_RETRUN_MODE)
    {
        log_info("%s robot mode set to low battery return mode",__FUNCTION__);
        run_mode_status_[LOW_BAT_RETRUN_MODE_INDEX] = status;
    }
    else if(robot_mode == IDLE_RETURN_MODE)
    {
        log_info("%s robot mode set to idle return mode",__FUNCTION__);
        run_mode_status_[IDLE_RETURN_MODE_INDEX] = status;
    }
    else
    {
        log_error("%s unrecognized robot run mode", __FUNCTION__);
    }
}

void Diagnostics::sysDiagDelayProc(void)
{
    log_warn("------------------%s----------------",__FUNCTION__);
    system_delay_check_ = true;
}

void Diagnostics::robotConnectStateCb(const atris_msgs::NetworkState &msg)
{
    boost::unique_lock<boost::mutex> lock(connect_state_mutex_);
    mqtt_connect_state_ = msg.network_state;
    // if the state is connected we need to start to report stored event to web(if there is any)
    if(mqtt_connect_state_ == 1)
    {
        log_info("%s network connected... , check if we have stored event in database\r\n",__FUNCTION__);
        sendDeviceWarning(0);
        connect_state_cond_.notify_all();
    }
    else
    {
        log_error("%s network disconnected!!!",__FUNCTION__);
        sendDeviceWarning(1);
    }
}

int Diagnostics::getConnectState(void)
{
    boost::unique_lock<boost::mutex> lock(connect_state_mutex_);
    return mqtt_connect_state_;
}

void Diagnostics::sendDeviceWarning(int on_off)
{
    log_info("%s",__FUNCTION__);
    atris_msgs::PeripheralControl req;
    if(on_off)
    {
        req.request.cmd = atris_msgs::PeripheralControl::Request::LIGHT_COLOR_CONTROL;
        req.request.color = atris_msgs::PeripheralControl::Request::LIGHT_CONTROL_RED;
        if (peripheral_ctrl_client_.call(req)) {
            if(req.response.result)
            {
                //log_info("%s light control warning success...",__FUNCTION__);
            }
            else
            {
                log_error("%s light control warning failed!!!",__FUNCTION__);
            }
        }
    }
    else
    {
        req.request.cmd = atris_msgs::PeripheralControl::Request::LIGHT_CONTROL_RELEASE;
        if (peripheral_ctrl_client_.call(req)) {
            if(req.response.result)
            {
                //log_info("%s light control release success...",__FUNCTION__);
            }
            else
            {
                log_error("%s light control release failed!!!",__FUNCTION__);
            }
        }
    }

}

float Diagnostics::floatTrim(float val)
{
    char str_f[100];
    sprintf(str_f, "%.2f", val);
    std::stringstream ss;
    ss << str_f;
    float f2;
    ss >> f2;
    return f2;
}

double Diagnostics::doubleTrim(double val)
{
    char str_d[100];
    sprintf(str_d, "%.2lf", val);
    std::stringstream ss;
    ss << str_d;
    double d2;
    ss >> d2;
    return d2;
}

void Diagnostics::notifyDeviceWarningEventNew(const std::string & event_content , const int & status, const uint64_t & timestamp, const std::string & serial_number, const std::string type, const int32_t value)
{
    Json::Value content;
    Json::Value evt_arr;
    Json::Value device_event_msg;
    std::string title;
    Json::FastWriter writer;
    std::string event_content_str = "";
    int iRet;

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string accid = std::string(shmrbt.robot.accid);

    content["category"] = "custom_event"; // custom event
    content["eventId"] = get_event_id_by_content(event_content);
    title = "notify_event_device_warning";
    // accid
    content["userId"] = accid;

    // timestamp
    content["recordedAt"] = timestamp;
    // segmentaition is NULL
    content["segmentation"] = "";

    content["customSegmentation"]["status"] = status;

    content["customSegmentation"]["serialNum"] = serial_number;
    content["customSegmentation"]["level"] = get_warn_level_by_content(event_content); // get event warn level

    // reason
    content["customSegmentation"]["reason"]["type"] = type;
    content["customSegmentation"]["reason"]["value"] = value;

    evt_arr.append(content);
    event_content_str = writer.write(evt_arr);
    log_info("%s event_content string : %s",__FUNCTION__, event_content_str.c_str());

    device_event_msg["event_message"] = evt_arr; // put the event content into event message field to parse
    log_info("%s publish message out", __FUNCTION__);

    iRet = checkCanSend(title, content["eventId"].asString(), device_event_msg);
    if(0 > iRet)
    {
        log_warn("%s not ready to send event msg, temporarily store it to database",__FUNCTION__);
    }
    else
    {
        log_info("%s ok to send robot event message , post it to web",__FUNCTION__);
    }

}

void Diagnostics::checkBatteryStartup(const int32_t &status)
{
    tinyros::Time now = tinyros::Time::now();

    log_info("%s, status:%d",__FUNCTION__, status);
    diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, status, "battery_cell_pressure_large", "");
    diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, status, "battery_hight_temp", "");
    diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, status, "battery_low_temp", "");
    diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, status, "battery_low_power", "");
    diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, status, "battery_severe_low_power", "");
    diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, status, "battery_under_vol", "");
    diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, status, "battery_over_discharge", "");
}

void Diagnostics::checkBatteryWarningEvent(int32_t low, int32_t mid, int32_t hight)
{
    static bool start_up = false;
    static bool cell_pres_large_send = false;
    static bool hight_temp_send = false;
    static bool low_temp_send = false;
    static bool low_power_send = false;
    static bool severe_low_power_send = false;
    static bool under_vol_send = false;
    static bool over_discharge_send = false;
    int32_t cell_pres_large         = (low >> BATTERY_ALARM_CELL_PRESSURE_LARGE_BIT) & BATTERY_ALARM_MASK_2_BIT;
    int32_t charge_hight_temp       = (low >> BATTERY_ALARM_CHARGING_HIGHT_TEMP_BIT) & BATTERY_ALARM_MASK_2_BIT;
    int32_t charge_low_temp         = (low >> BATTERY_ALARM_CHARGING_LOW_TEMP_BIT) & BATTERY_ALARM_MASK_2_BIT;
    int32_t discharge_hight_temp    = (low >> BATTERY_ALARM_DISCHARGING_HIGHT_TEMP_BIT) & BATTERY_ALARM_MASK_2_BIT;
    int32_t discharge_low_temp      = (low >> BATTERY_ALARM_DISCHARGING_LOW_TEMP_BIT) & BATTERY_ALARM_MASK_2_BIT;
    int32_t low_power               = (mid >> BATTERY_ALARM_LOW_POWR_BIT) & BATTERY_ALARM_MASK_2_BIT;
    int32_t under_vol               = (mid >> BATTERY_ALARM_UNDER_VOLTAGE_BIT) & BATTERY_ALARM_MASK_2_BIT;
    int32_t over_discharge          = (mid >> BATTERY_ALARM_OVER_DISCHARGE_BIT) & BATTERY_ALARM_MASK_2_BIT;
    static std::string cell_pressure_large_serial;
    static std::string charge_hight_temp_serial;
    static std::string charge_low_temp_serial;
    static std::string low_power_serial;
    static std::string severe_low_power_serial;
    static std::string under_vol_serial;
    static std::string over_discharge_serial;

    char rand_str[16] = {0};
    DataBaseCmdStru status_cmd_stru;

    //char rand_str[16] = {0};
    //Utils::get_instance()->getRandStr(rand_str);
    //std::string serial_number = rand_str;

    if (!start_up)
    {
        if (cell_pres_large == 0 && charge_hight_temp == 0 && charge_low_temp == 0 && discharge_hight_temp == 0 && discharge_low_temp == 0 
             && low_power == 0 && under_vol == 0 && over_discharge == 0)
        {
            checkBatteryStartup(0);
        }
        start_up = true;
    }

    tinyros::Time now = tinyros::Time::now();

    if ((low != 0 || mid != 0 || hight != 0) || (cell_pres_large_send || hight_temp_send ||low_power_send || severe_low_power_send || under_vol_send || over_discharge_send))
    {
        log_info("%s , low:%d, mid:%d, hight:%d,",__FUNCTION__, low, mid, hight);
    }
    
    //电池电芯压差大
    if (cell_pres_large != 0 && !cell_pres_large_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        cell_pressure_large_serial = rand_str;

        notifyDeviceWarningEventNew("battery_cell_pressure_large", 1, (uint64_t)now.toSec(), cell_pressure_large_serial, "code", 0);
        cell_pres_large_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "battery_cell_pressure_large", cell_pressure_large_serial);
    }
    else if (cell_pres_large == 0 && cell_pres_large_send)
    {
        notifyDeviceWarningEventNew("battery_cell_pressure_large", 0, (uint64_t)now.toSec(), cell_pressure_large_serial, "code", 0);
        cell_pres_large_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "battery_cell_pressure_large", cell_pressure_large_serial);
    }

    //电池温度过热
    if ((charge_hight_temp != 0 || discharge_hight_temp != 0)  && !hight_temp_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        charge_hight_temp_serial = rand_str;

        notifyDeviceWarningEventNew("battery_hight_temp", 1, (uint64_t)now.toSec(), charge_hight_temp_serial, "code", 1);
        hight_temp_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "battery_hight_temp", charge_hight_temp_serial);
    }
    else if (charge_hight_temp == 0 && discharge_hight_temp == 0 && hight_temp_send)
    {
        notifyDeviceWarningEventNew("battery_hight_temp", 0, (uint64_t)now.toSec(), charge_hight_temp_serial, "code", 1);
        hight_temp_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "battery_hight_temp", charge_hight_temp_serial);
    }

    //电池温度过低
    if ((charge_low_temp != 0 || discharge_low_temp != 0)  && !low_temp_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        charge_low_temp_serial = rand_str;

        notifyDeviceWarningEventNew("battery_low_temp", 1, (uint64_t)now.toSec(), charge_low_temp_serial, "code", 2);
        low_temp_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "battery_low_temp", charge_low_temp_serial);
    }
    else if (charge_low_temp == 0 && discharge_low_temp == 0 && low_temp_send)
    {
        notifyDeviceWarningEventNew("battery_low_temp", 0, (uint64_t)now.toSec(), charge_low_temp_serial, "code", 2);
        low_temp_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "battery_low_temp", charge_low_temp_serial);
    }

    //低电量
    if (low_power == 1 && !low_power_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        low_power_serial = rand_str;

        notifyDeviceWarningEventNew("battery_low_power", 1, (uint64_t)now.toSec(), low_power_serial, "code", 3);
        low_power_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "battery_low_power", low_power_serial);
    }

    //严重低电量
    if (low_power == 2 && !severe_low_power_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        severe_low_power_serial = rand_str;

        notifyDeviceWarningEventNew("battery_severe_low_power", 1, (uint64_t)now.toSec(), severe_low_power_serial, "code", 4);
        severe_low_power_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "battery_severe_low_power", severe_low_power_serial);
    }

    if (low_power == 0 && (low_power_send || severe_low_power_send))
    {
        if (low_power_send)
        {
            notifyDeviceWarningEventNew("battery_low_power", 0, (uint64_t)now.toSec(), low_power_serial, "code", 3);
            low_power_send = false;

            diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "battery_low_power", low_power_serial);
        }
        
        if (severe_low_power_send)
        {
            notifyDeviceWarningEventNew("battery_severe_low_power", 0, (uint64_t)now.toSec(), severe_low_power_serial, "code", 4);
            severe_low_power_send = false;

            diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "battery_severe_low_power", severe_low_power_serial);
        }
        
    }

    //电池欠压
    if (under_vol != 0 && !under_vol_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        under_vol_serial = rand_str;

        notifyDeviceWarningEventNew("battery_under_vol", 1, (uint64_t)now.toSec(), under_vol_serial, "code", 5);
        under_vol_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "battery_under_vol", under_vol_serial); 
    }
    else if (under_vol == 0 && under_vol_send)
    {
        notifyDeviceWarningEventNew("battery_under_vol", 0, (uint64_t)now.toSec(), under_vol_serial, "code", 5);
        under_vol_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "battery_under_vol", under_vol_serial); 
    }

    //电池过放
    if (over_discharge != 0 && !over_discharge_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        over_discharge_serial = rand_str;

        notifyDeviceWarningEventNew("battery_over_discharge", 1, (uint64_t)now.toSec(), over_discharge_serial, "code", 6);
        over_discharge_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "battery_over_discharge", over_discharge_serial); 
    }
    else if (over_discharge == 0 && over_discharge_send)
    {
        notifyDeviceWarningEventNew("battery_over_discharge", 0, (uint64_t)now.toSec(), over_discharge_serial, "code", 6);
        over_discharge_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "battery_over_discharge", over_discharge_serial); 
    }
}

void Diagnostics::diagEventCmdAddCycle(const int32_t &cmd_type, const tinyros::Time &now, int32_t status, const std::string &event_content, const std::string & serial_num)
{
    DataBaseCmdStru status_cmd_stru;

    status_cmd_stru.cmd_type = cmd_type;
    status_cmd_stru.event_data.event_timestamp = (uint64_t)(now.toSec());
    status_cmd_stru.event_data.data_item.event_status = status;
    status_cmd_stru.event_data.data_item.event_content = event_content; // erro event content field
    status_cmd_stru.event_data.data_item.event_serial_num = serial_num; 
    // save it to database
    diagEventCmdAdd(status_cmd_stru);
}
/*
* #  异常码   		异常说明
* #	1				编码器ABZ报警
* #	2				编码器UVW报警
* #	3				位置超差
* #	4				失速
* #	5				ADC零点异常
* #	6				过载
* #	7				功率电源欠压
* #	8				功率电源过压
* #	9				过流
* #	A				瞬时放电报警
* #	B				平均放电报警
* #	C				参数读写异常
* #	D				输入端口重复定义
* #	E				断线保护
* #	F				温度报警
*/
void Diagnostics::checkDriverLeftFrontWarningEvent(const tinyros::atris_msgs::chassis_abnor_info &msg)
{
    if(strcmp(msg.axle,"leftFront") != 0)
        return;

    static bool motor_overload_sent = false;
    static bool motor_fault_send = false;
    static bool elect_fault_send = false;
    static bool encoder_fault_send = false;
    static std::string motor_overload_serial;
    static std::string motor_fault_serial;
    static std::string elect_fault_serial;
    static std::string encoder_fault_serial;
    char rand_str[16] = {0};
    DataBaseCmdStru status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();

    int32_t steer_error_code = msg.steer_error_code;
    int32_t direct_error_code = msg.direct_error_code;
    int32_t chassis_abnor = msg.chassis_abnor;

    if (chassis_abnor != 0 || motor_overload_sent || motor_fault_send || elect_fault_send || encoder_fault_send)
    {
        log_info("%s leftFront, chassis_abnor:%d, steer_error_code:%d, direct_error_code:%d",__FUNCTION__, chassis_abnor, steer_error_code, direct_error_code);
    }

    //电机过载
    if (chassis_abnor != 0 && (steer_error_code == 6 || direct_error_code == 6) && !motor_overload_sent)
    {
        Utils::get_instance()->getRandStr(rand_str);
        motor_overload_serial = rand_str;

        log_info("%s, driver_left_front_motor_overload[0]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_left_front_motor_overload", 1, (uint64_t)now.toSec(), motor_overload_serial, "code", 0);
        motor_overload_sent = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_left_front_motor_overload", motor_overload_serial);
    }
    else if (chassis_abnor == 0 && motor_overload_sent)
    {
        notifyDeviceWarningEventNew("driver_left_front_motor_overload", 0, (uint64_t)now.toSec(), motor_overload_serial, "code", 0);
        motor_overload_sent = false;

        log_info("%s, driver_left_front_motor_overload[0]: status：0", __FUNCTION__);

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_left_front_motor_overload", motor_overload_serial); 
    }

    //电机故障
    if (chassis_abnor != 0 && (steer_error_code == 1 || steer_error_code == 2 || steer_error_code == 0xF 
        || direct_error_code == 1 || direct_error_code == 2 || direct_error_code == 0xF) && !motor_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        motor_fault_serial = rand_str;

        log_info("%s, driver_left_front_motor_fault[1]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_left_front_motor_fault", 1, (uint64_t)now.toSec(), motor_fault_serial, "code", 1);
        motor_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_left_front_motor_fault", motor_fault_serial);
    }
    else if (chassis_abnor == 0 && motor_fault_send)
    {
        log_info("%s, driver_left_front_motor_fault[1]: status：0", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_left_front_motor_fault", 0, (uint64_t)now.toSec(), motor_fault_serial, "code", 1);
        motor_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_left_front_motor_fault", motor_fault_serial);
    }

    //电驱故障
    if (chassis_abnor != 0 && ((steer_error_code >=3 && steer_error_code <= 5) || (steer_error_code >=7 && steer_error_code <= 0xE)
         || (direct_error_code >=3 && direct_error_code <= 5) || (direct_error_code >=7 && direct_error_code <= 0xE)) && !elect_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        elect_fault_serial = rand_str;

        log_info("%s, driver_left_front_elect_fault[2]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_left_front_elect_fault", 1, (uint64_t)now.toSec(), elect_fault_serial, "code", 2);
        elect_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_left_front_elect_fault", elect_fault_serial);
    }
    else if (chassis_abnor == 0 && elect_fault_send)
    {
        log_info("%s, driver_left_front_elect_fault[2]: status：0", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_left_front_elect_fault", 0, (uint64_t)now.toSec(), elect_fault_serial, "code", 2);
        elect_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_left_front_elect_fault", elect_fault_serial);
    }

    //编码器故障
    if (chassis_abnor != 0 && (steer_error_code == 1 || steer_error_code == 2 || direct_error_code == 1 || direct_error_code == 2) && !encoder_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        encoder_fault_serial = rand_str;

        log_info("%s, driver_left_front_encoder_fault[3]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_left_front_encoder_fault", 1, (uint64_t)now.toSec(), encoder_fault_serial, "code", 3);
        encoder_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_left_front_encoder_fault", encoder_fault_serial);
    }
    else if (chassis_abnor == 0 && encoder_fault_send)
    {
        log_info("%s, driver_left_front_encoder_fault[3]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_left_front_encoder_fault", 0, (uint64_t)now.toSec(), encoder_fault_serial, "code", 3);
        encoder_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_left_front_encoder_fault", encoder_fault_serial);
    }

}

void Diagnostics::checkDriverRightFrontWarningEvent(const tinyros::atris_msgs::chassis_abnor_info &msg)
{
    if(strcmp(msg.axle,"rightFront") != 0)
        return;

    static bool motor_overload_sent = false;
    static bool motor_fault_send = false;
    static bool elect_fault_send = false;
    static bool encoder_fault_send = false;
    static std::string motor_overload_serial;
    static std::string motor_fault_serial;
    static std::string elect_fault_serial;
    static std::string encoder_fault_serial;
    char rand_str[16] = {0};
    DataBaseCmdStru status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();

    int32_t steer_error_code = msg.steer_error_code;
    int32_t direct_error_code = msg.direct_error_code;
    int32_t chassis_abnor = msg.chassis_abnor;

    if (chassis_abnor != 0 || motor_overload_sent || motor_fault_send || elect_fault_send || encoder_fault_send)
    {
        log_info("%s rightFront, chassis_abnor:%d, steer_error_code:%d, direct_error_code:%d",__FUNCTION__, chassis_abnor, steer_error_code, direct_error_code);
    }

    //电机过载
    if (chassis_abnor != 0 && (steer_error_code == 6 || direct_error_code == 6) && !motor_overload_sent)
    {
        Utils::get_instance()->getRandStr(rand_str);
        motor_overload_serial = rand_str;

        log_info("%s, driver_right_front_motor_overload[4]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_right_front_motor_overload", 1, (uint64_t)now.toSec(), motor_overload_serial, "code", 4);
        motor_overload_sent = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_right_front_motor_overload", motor_overload_serial);
    }
    else if (chassis_abnor == 0 && motor_overload_sent)
    {
        log_info("%s, driver_right_front_motor_overload[4]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_front_motor_overload", 0, (uint64_t)now.toSec(), motor_overload_serial, "code", 4);
        motor_overload_sent = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_right_front_motor_overload", motor_overload_serial); 
    }

    //电机故障
    if (chassis_abnor != 0 && (steer_error_code == 1 || steer_error_code == 2 || steer_error_code == 0xF 
        || direct_error_code == 1 || direct_error_code == 2 || direct_error_code == 0xF) && !motor_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        motor_fault_serial = rand_str;

        log_info("%s, driver_right_front_motor_fault[5]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_right_front_motor_fault", 1, (uint64_t)now.toSec(), motor_fault_serial, "code", 5);
        motor_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_right_front_motor_fault", motor_fault_serial);
    }
    else if (chassis_abnor == 0 && motor_fault_send)
    {
        log_info("%s, driver_right_front_motor_fault[5]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_front_motor_fault", 0, (uint64_t)now.toSec(), motor_fault_serial, "code", 5);
        motor_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_right_front_motor_fault", motor_fault_serial);
    }

    //电驱故障
    if (chassis_abnor != 0 && ((steer_error_code >=3 && steer_error_code <= 5) || (steer_error_code >=7 && steer_error_code <= 0xE)
         || (direct_error_code >=3 && direct_error_code <= 5) || (direct_error_code >=7 && direct_error_code <= 0xE)) && !elect_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        elect_fault_serial = rand_str;

        log_info("%s, driver_right_front_elect_fault[6]: status：1", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_front_elect_fault", 1, (uint64_t)now.toSec(), elect_fault_serial, "code", 6);
        elect_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_right_front_elect_fault", elect_fault_serial);
    }
    else if (chassis_abnor == 0 && elect_fault_send)
    {
        log_info("%s, driver_right_front_elect_fault[6]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_front_elect_fault", 0, (uint64_t)now.toSec(), elect_fault_serial, "code", 6);
        elect_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_right_front_elect_fault", elect_fault_serial);
    }

    //编码器故障
    if (chassis_abnor != 0 && (steer_error_code == 1 || steer_error_code == 2 || direct_error_code == 1 || direct_error_code == 2) && !encoder_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        encoder_fault_serial = rand_str;

        log_info("%s, driver_right_front_encoder_fault[7]: status：1", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_front_encoder_fault", 1, (uint64_t)now.toSec(), encoder_fault_serial, "code", 7);
        encoder_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_right_front_encoder_fault", encoder_fault_serial);
    }
    else if (chassis_abnor == 0 && encoder_fault_send)
    {
        log_info("%s, driver_right_front_encoder_fault[7]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_front_encoder_fault", 0, (uint64_t)now.toSec(), encoder_fault_serial, "code", 7);
        encoder_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_right_front_encoder_fault", encoder_fault_serial);
    }
}

void Diagnostics::checkDriverLeftRearWarningEvent(const tinyros::atris_msgs::chassis_abnor_info &msg)
{
    if(strcmp(msg.axle,"leftRear") != 0)
        return;

    static bool motor_overload_sent = false;
    static bool motor_fault_send = false;
    static bool elect_fault_send = false;
    static bool encoder_fault_send = false;
    static std::string motor_overload_serial;
    static std::string motor_fault_serial;
    static std::string elect_fault_serial;
    static std::string encoder_fault_serial;
    char rand_str[16] = {0};
    DataBaseCmdStru status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();

    int32_t steer_error_code = msg.steer_error_code;
    int32_t direct_error_code = msg.direct_error_code;
    int32_t chassis_abnor = msg.chassis_abnor;

    if (chassis_abnor != 0 || motor_overload_sent || motor_fault_send || elect_fault_send || encoder_fault_send)
    {
        log_info("%s leftRear, chassis_abnor:%d, steer_error_code:%d, direct_error_code:%d",__FUNCTION__, chassis_abnor, steer_error_code, direct_error_code);
    }

    //电机过载
    if (chassis_abnor != 0 && (steer_error_code == 6 || direct_error_code == 6) && !motor_overload_sent)
    {
        Utils::get_instance()->getRandStr(rand_str);
        motor_overload_serial = rand_str;

        log_info("%s, driver_left_rear_motor_overload[8]: status：1", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_left_rear_motor_overload", 1, (uint64_t)now.toSec(), motor_overload_serial, "code", 8);
        motor_overload_sent = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_left_rear_motor_overload", motor_overload_serial);
    }
    else if (chassis_abnor == 0 && motor_overload_sent)
    {
        log_info("%s, driver_left_rear_motor_overload[8]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_left_rear_motor_overload", 0, (uint64_t)now.toSec(), motor_overload_serial, "code", 8);
        motor_overload_sent = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_left_rear_motor_overload", motor_overload_serial); 
    }

    //电机故障
    if (chassis_abnor != 0 && (steer_error_code == 1 || steer_error_code == 2 || steer_error_code == 0xF 
        || direct_error_code == 1 || direct_error_code == 2 || direct_error_code == 0xF) && !motor_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        motor_fault_serial = rand_str;

        log_info("%s, driver_left_rear_motor_fault[9]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_left_rear_motor_fault", 1, (uint64_t)now.toSec(), motor_fault_serial, "code", 9);
        motor_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_left_rear_motor_fault", motor_fault_serial);
    }
    else if (chassis_abnor == 0 && motor_fault_send)
    {
        log_info("%s, driver_left_rear_motor_fault[9]: status：0", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_left_rear_motor_fault", 0, (uint64_t)now.toSec(), motor_fault_serial, "code", 9);
        motor_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_left_rear_motor_fault", motor_fault_serial);
    }

    //电驱故障
    if (chassis_abnor != 0 && ((steer_error_code >=3 && steer_error_code <= 5) || (steer_error_code >=7 && steer_error_code <= 0xE)
         || (direct_error_code >=3 && direct_error_code <= 5) || (direct_error_code >=7 && direct_error_code <= 0xE)) && !elect_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        elect_fault_serial = rand_str;

        log_info("%s, driver_left_rear_elect_fault[10]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_left_rear_elect_fault", 1, (uint64_t)now.toSec(), elect_fault_serial, "code", 10);
        elect_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_left_rear_elect_fault", elect_fault_serial);
    }
    else if (chassis_abnor == 0 && elect_fault_send)
    {
        log_info("%s, driver_left_rear_elect_fault[10]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_left_rear_elect_fault", 0, (uint64_t)now.toSec(), elect_fault_serial, "code", 10);
        elect_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_left_rear_elect_fault", elect_fault_serial);
    }

    //编码器故障
    if (chassis_abnor != 0 && (steer_error_code == 1 || steer_error_code == 2 || direct_error_code == 1 || direct_error_code == 2) && !encoder_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        encoder_fault_serial = rand_str;

        log_info("%s, driver_left_rear_encoder_fault[11]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_left_rear_encoder_fault", 1, (uint64_t)now.toSec(), encoder_fault_serial, "code", 11);
        encoder_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_left_rear_encoder_fault", encoder_fault_serial);
    }
    else if (chassis_abnor == 0 && encoder_fault_send)
    {
        log_info("%s, driver_left_rear_encoder_fault[11]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_left_rear_encoder_fault", 0, (uint64_t)now.toSec(), encoder_fault_serial, "code", 11);
        encoder_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_left_rear_encoder_fault", encoder_fault_serial);
    }
}

void Diagnostics::checkDriverRightRearWarningEvent(const tinyros::atris_msgs::chassis_abnor_info &msg)
{
    if(strcmp(msg.axle,"rightRear") != 0)
        return;

    static bool motor_overload_sent = false;
    static bool motor_fault_send = false;
    static bool elect_fault_send = false;
    static bool encoder_fault_send = false;
    static std::string motor_overload_serial;
    static std::string motor_fault_serial;
    static std::string elect_fault_serial;
    static std::string encoder_fault_serial;
    char rand_str[16] = {0};
    DataBaseCmdStru status_cmd_stru;
    tinyros::Time now = tinyros::Time::now();

    int32_t steer_error_code = msg.steer_error_code;
    int32_t direct_error_code = msg.direct_error_code;
    int32_t chassis_abnor = msg.chassis_abnor;

    if (chassis_abnor != 0 || motor_overload_sent || motor_fault_send || elect_fault_send || encoder_fault_send)
    {
        log_info("%s rightRear, chassis_abnor:%d, steer_error_code:%d, direct_error_code:%d",__FUNCTION__, chassis_abnor, steer_error_code, direct_error_code);
    }

    //电机过载
    if (chassis_abnor != 0 && (steer_error_code == 6 || direct_error_code == 6) && !motor_overload_sent)
    {
        Utils::get_instance()->getRandStr(rand_str);
        motor_overload_serial = rand_str;

        log_info("%s, driver_right_rear_motor_overload[12]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_right_rear_motor_overload", 1, (uint64_t)now.toSec(), motor_overload_serial, "code", 12);
        motor_overload_sent = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_right_rear_motor_overload", motor_overload_serial);
    }
    else if (chassis_abnor == 0 && motor_overload_sent)
    {
        log_info("%s, driver_right_rear_motor_overload[12]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_rear_motor_overload", 0, (uint64_t)now.toSec(), motor_overload_serial, "code", 12);
        motor_overload_sent = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_right_rear_motor_overload", motor_overload_serial); 
    }

    //电机故障
    if (chassis_abnor != 0 && (steer_error_code == 1 || steer_error_code == 2 || steer_error_code == 0xF 
        || direct_error_code == 1 || direct_error_code == 2 || direct_error_code == 0xF) && !motor_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        motor_fault_serial = rand_str;

        log_info("%s, driver_right_rear_motor_fault[13]: status：1", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_rear_motor_fault", 1, (uint64_t)now.toSec(), motor_fault_serial, "code", 13);
        motor_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_right_rear_motor_fault", motor_fault_serial);
    }
    else if (chassis_abnor == 0 && motor_fault_send)
    {
        log_info("%s, driver_right_rear_motor_fault[13]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_rear_motor_fault", 0, (uint64_t)now.toSec(), motor_fault_serial, "code", 13);
        motor_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_right_rear_motor_fault", motor_fault_serial);
    }

    //电驱故障
    if (chassis_abnor != 0 && ((steer_error_code >=3 && steer_error_code <= 5) || (steer_error_code >=7 && steer_error_code <= 0xE)
         || (direct_error_code >=3 && direct_error_code <= 5) || (direct_error_code >=7 && direct_error_code <= 0xE)) && !elect_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        elect_fault_serial = rand_str;

        log_info("%s, driver_right_rear_elect_fault[14]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_right_rear_elect_fault", 1, (uint64_t)now.toSec(), elect_fault_serial, "code", 14);
        elect_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_right_rear_elect_fault", elect_fault_serial);
    }
    else if (chassis_abnor == 0 && elect_fault_send)
    {
        log_info("%s, driver_right_rear_elect_fault[14]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_rear_elect_fault", 0, (uint64_t)now.toSec(), elect_fault_serial, "code", 14);
        elect_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "driver_right_rear_elect_fault", elect_fault_serial);
    }

    //编码器故障
    if (chassis_abnor != 0 && (steer_error_code == 1 || steer_error_code == 2 || direct_error_code == 1 || direct_error_code == 2) && !encoder_fault_send)
    {
        Utils::get_instance()->getRandStr(rand_str);
        encoder_fault_serial = rand_str;

        log_info("%s, driver_right_rear_encoder_fault[15]: status：1", __FUNCTION__);

        notifyDeviceWarningEventNew("driver_right_rear_encoder_fault", 1, (uint64_t)now.toSec(), encoder_fault_serial, "code", 15);
        encoder_fault_send = true;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_right_rear_encoder_fault", encoder_fault_serial);
    }
    else if (chassis_abnor == 0 && encoder_fault_send)
    {
        log_info("%s, driver_right_rear_encoder_fault[15]: status：0", __FUNCTION__);
        notifyDeviceWarningEventNew("driver_right_rear_encoder_fault", 0, (uint64_t)now.toSec(), encoder_fault_serial, "code", 15);
        encoder_fault_send = false;

        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "driver_right_rear_encoder_fault", encoder_fault_serial);
    }
}

void Diagnostics::checkDriverWarningEventStartup(const tinyros::atris_msgs::chassis_abnor_info &msg)
{
    static bool left_front = false;
    static bool right_front = false;
    static bool left_rear = false;
    static bool right_rear = false;
    int32_t chassis_abnor = msg.chassis_abnor;

    if (left_front && right_front && left_rear && right_rear)
    {
        return;
    }

    tinyros::Time now = tinyros::Time::now();

    if(chassis_abnor == 0 && strcmp(msg.axle,"leftFront") == 0)
    {
        left_front = true;
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_left_front_motor_overload", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_left_front_motor_fault", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_left_front_elect_fault", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_left_front_encoder_fault", "");
    }
    else if (chassis_abnor == 0 && strcmp(msg.axle,"rightFront") == 0)
    {
        right_front = true;
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_right_front_motor_overload", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_right_front_motor_fault", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_right_front_elect_fault", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_right_front_encoder_fault", "");
    }
    else if (chassis_abnor == 0 && strcmp(msg.axle,"leftRear") == 0)
    {
        left_rear = true;
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_left_rear_motor_overload", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_left_rear_motor_fault", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_left_rear_elect_fault", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_left_rear_encoder_fault", "");
    }
    else if (chassis_abnor == 0 && strcmp(msg.axle,"rightRear") == 0)
    {
        right_rear = true;
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_right_rear_motor_overload", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_right_rear_motor_fault", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_right_rear_elect_fault", "");
        diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "driver_right_rear_encoder_fault", "");
    }
}


void Diagnostics::checkCollisionEvent(int32_t status)
{
    static bool start_up = false;
    static bool collision_sent = false;
    static std::string collision_serial;
    char rand_str[16] = {0};
    tinyros::Time now = tinyros::Time::now();

    if (!start_up)
    {
        start_up = true;
        if (status == 0)
        {
            log_info("%s, startUp, status:%d", __FUNCTION__, status);
            diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, "collision_warnning", "");
        }
    }

    if (status != 0 && !collision_sent)
    {
        collision_sent = true;

        Utils::get_instance()->getRandStr(rand_str);
        collision_serial = rand_str;

        log_info("%s, status:%d", __FUNCTION__, status);

        notifyDeviceWarningEvent("collision_warnning", 1, (uint64_t)now.toSec(), collision_serial);
        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 1, "collision_warnning", collision_serial);
    }
    else if (status == 0 &&  collision_sent)
    {
        collision_sent = false;

        log_info("%s, status:%d", __FUNCTION__, status);

        notifyDeviceWarningEvent("collision_warnning", 0, (uint64_t)now.toSec(), collision_serial);
        diagEventCmdAddCycle(EVENT_PROC_THIS_CYCLE, now, 0, "collision_warnning", collision_serial);
    }
}

void Diagnostics::checkUltraStartup()
{
    static bool start_up = false;

    if (start_up)
        return;

    tinyros::Time now = tinyros::Time::now();
    start_up = true;
    
    for (int i = 0; i < 4; i++)
    {
        std::string event_content = "ultra_error_";
        event_content = event_content + std::to_string(i);
        int32_t data = diag_["robot_info"]["ultrasound"]["data"][i].asInt();
        if (data < 1000)
        {
            diagEventCmdAddCycle(EVENT_CHECK_LAST_CYCLE, now, 0, event_content, "");
        }
    }
}
