/*
 * diagnostics.h
 *
 *  Created on: 2018-6-25
 *      Author: fupj
 */

#ifndef DIAGNOSTICS_H_
#define DIAGNOSTICS_H_
#include <boost/thread.hpp>
#include <json/json.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "atris_msgs/SignalMessage.h"
#include "tiny_ros/ros.h"
#include <ros/ros.h>
#include "atris_defines.h"
#include "atris_msgs/AisoundTTS.h"
#include "atris_msgs/RobotInfo.h"
#include "atris_msgs/RobotBraked.h"
#include "atris_msgs/RebootBraked.h"
#include "atris_msgs/ProtectMode.h"
#include "atris_msgs/McuLogCtrl.h"
#include "atris_msgs/GetVoiceChatStatus.h"
#include "atris_msgs/GetLampStatus.h"
#include "atris_msgs/GetStandbyMode.h"
#include "atris_msgs/GetPPPlayingStatus.h"
#include "atris_msgs/GetDisperseStatus.h"
#include "atris_msgs/GetPatrolStatus.h"
#include "atris_msgs/GetPtzStatus.h"
#include "atris_msgs/GetSwVersion.h"
#include "atris_msgs/GetChassisInfo.h"
#include "atris_msgs/GetOTAPreCondition.h"
#include "atris_msgs/RobotImu.h"
#include "tiny_ros/atris_msgs/RobotPose.h"
#include "tiny_ros/atris_msgs/chassis_abnor_info.h"
#include "atris_msgs/PeripheralControl.h"
#include "std_msgs/Int32.h"

#include "atris_msgs/CanData.h"
#include "atris_msgs/GetDiagnosticsInfo.h"
#include "atris_msgs/GetGpsPos.h"
#include "atris_msgs/MiddleFanControl.h"
#include "atris_msgs/UpgradeStatus.h"
#include "list.h"
#include "diag_helper.h"
#include "utils/utils.h"
#include "atris_msgs/RobotRunMode.h"
#include "atris_msgs/NetworkState.h"

#include "CTimer.hpp"
using boost::asio::ip::udp;

#define EVENT_MSG_LIST_SIZE_MAX 50 // we cached at list 50 event messages to process
#define SWUPGRADE_VERSION_FILE "/home/atris/atris_app/config/versionInfo.txt"
#define MONITOR_LOG_FILE_FOLDER_PATH "/upload/mcu_log/monitor/"
#define CHASSIS_LOG_FILE_FOLDER_PATH "/upload/mcu_log/chassis_controller/"
#define HFS_DIR_PATH "/home/atris/atris_app/bin/dist/hfs"

#define NUMBER_OF_MODE_PRIORITY 7

#define LIDAR_IP "192.168.1.200"
#define PTZ_IP "10.20.18.3"
#define CCB_IP "10.20.18.10"
#define SYS_IP "10.20.18.6"

#define LIDAR_ERROR_CNT 5
#define LIDAR_RECOVER_CNT 3

#define CCB_ERROR_CNT 5
#define CCB_RECOVER_CNT 3

#define SYS_ERROR_CNT 5
#define SYS_RECOVER_CNT 3

#define PTZ_ERROR_CNT 5
#define PTZ_RECOVER_CNT 3

/*
*  data_i[27]        告警信息低字节（16bit）
*  data_i[28]        告警信息中字节（16bit）
*  data_i[39]        告警信息高字节（16bit）
*  电池告警信息一共40比特
*/
//data_i[27]
#define BATTERY_ALARM_MASK_2_BIT (3)
#define BATTERY_ALARM_MASK_1_BIT (1)
#define BATTERY_ALARM_CELL_PRESSURE_LARGE_BIT          (4)  //电池电芯压差大
#define BATTERY_ALARM_CHARGING_HIGHT_TEMP_BIT          (6)  //充电高温
#define BATTERY_ALARM_CHARGING_LOW_TEMP_BIT            (8)  //充电低温
#define BATTERY_ALARM_DISCHARGING_HIGHT_TEMP_BIT       (10)  //放电高温
#define BATTERY_ALARM_DISCHARGING_LOW_TEMP_BIT         (12)  //放电低温
/*
* data_i[28]
*/
#define BATTERY_ALARM_LOW_POWR_BIT                     (2)  //低电量、严重低电量,bit18,bit19
#define BATTERY_ALARM_UNDER_VOLTAGE_BIT                (6) //电池欠压,bit22,bit23
#define BATTERY_ALARM_OVER_DISCHARGE_BIT               (10) //电池过放,bit26,bit27

#define MAXIMUM_STORED_EVENT_NUM 50
#if 1
enum RobotRunMode
{
    IDLE_MODE = 0,
    TASK_MODE = 1,
    EMERGENCY_TASK_MODE = 2,
    BACKGROUND_CONTROL_MODE = 3,
    REMOTE_CONTROL_MODE = 4,
    LOW_BAT_RETRUN_MODE = 5,
    IDLE_RETURN_MODE = 6

};
#endif

enum RobotRunModeIndex
{
    REMOTE_CONTROL_MODE_INDEX = 0,
    LOW_BAT_RETRUN_MODE_INDEX = 1,
    BACKGROUND_CONTROL_MODE_INDEX = 2,
    EMERGENCY_TASK_MODE_INDEX = 3,
    TASK_MODE_INDEX = 4,
    IDLE_RETURN_MODE_INDEX = 5,
    IDLE_MODE_INDEX = 6
};

typedef struct __odom_info
{
    double pos_x;
    double pos_y;
    double pos_a;
    double speed_x;
    double speed_y;
    double speed_a;
    double dist;
} odom_info;

enum EventVarIndex
{
    LOCALIZATION_LOST_INDEX = 0,
    SIGNAL_BAD_INDEX = 1,
    PTZ_ERROR_INDEX = 2,
    LIDAR_ERROR_INDEX = 3,
    ULTRA_ERROR_INDEX = 4,
    ANTI_DROP_ERROR_INDEX = 5,
    IMU_ERROR_INDEX = 6,
    CHASSIS_ERROR_INDEX = 7,
    BATTERY_ERROR_INDEX = 8,
    SPEAKER_ERROR_INDEX = 9,
    MIC_ERROR_INDEX = 10,
    VOIP_ERROR_INDEX = 11,
    FLOOD_WARNING_INDEX = 12,
    OVERTEMP_WARNING_INDEX = 13,
    EMERGE_STOP_WARNING_INDEX = 14,
    COLLISION_WARNING_INDEX = 15,
    ABNORMAL_SHUTDOWN_WARNING_INDEX = 16,
    CCB_ERROR_INDEX = 17,
    SYS_ERROR_INDEX = 18,

    ULTRA_ERROR_INDEX_0 = 19,
    ULTRA_ERROR_INDEX_1 = 20,
    ULTRA_ERROR_INDEX_2 = 21,
    ULTRA_ERROR_INDEX_3 = 22,

    BATTERY_ERROR_INDEX_0 = 23,
    BATTERY_ERROR_INDEX_1 = 24,
    BATTERY_ERROR_INDEX_2 = 25,
    BATTERY_ERROR_INDEX_3 = 26,
    BATTERY_ERROR_INDEX_4 = 27,
    BATTERY_ERROR_INDEX_5 = 28,
    BATTERY_ERROR_INDEX_6 = 29,

    DRIVER_LEFT_FRONT_INDEX_0 = 30,
    DRIVER_LEFT_FRONT_INDEX_1 = 31,
    DRIVER_LEFT_FRONT_INDEX_2 = 32,
    DRIVER_LEFT_FRONT_INDEX_3 = 33,

    DRIVER_RIGHT_FRONT_INDEX_0 = 34,
    DRIVER_RIGHT_FRONT_INDEX_1 = 35,
    DRIVER_RIGHT_FRONT_INDEX_2 = 36,
    DRIVER_RIGHT_FRONT_INDEX_3 = 37,

    DRIVER_LEFT_REAR_INDEX_0 = 38,
    DRIVER_LEFT_REAR_INDEX_1 = 39,
    DRIVER_LEFT_REAR_INDEX_2 = 40,
    DRIVER_LEFT_REAR_INDEX_3 = 41,

    DRIVER_RIGHT_REAR_INDEX_0 = 42,
    DRIVER_RIGHT_REAR_INDEX_1 = 43,
    DRIVER_RIGHT_REAR_INDEX_2 = 44,
    DRIVER_RIGHT_REAR_INDEX_3 = 45,

    EVENT_VAR_INDEX_MAX = 46

};

enum DatabaseCommand
{
    EVENT_CHECK_LAST_CYCLE = 0, // check event that is happened last cycle, see if it is recovered
    EVENT_PROC_THIS_CYCLE = 1 // check event which is happened this cycle, and report the event
};

enum EmergeBreakTriggerSource
{
    EMERGE_BREAK_NULL = 0,
    EMERGE_BREAK_BUTTON = 1,
    EMERGE_BREAK_CAN = 2
};

enum BumperTriggerSource
{
    BUMPER_TRIGGER_NULL = 0,
    BUMPER_TRIGGER_FRONT = 1,
    BUMPER_TRIGGER_TEAR = 2
};

enum WarningEventType
{
    NAVIGATION_WARNING = 0,
    COMMUNICATION_WARNING = 1,
    DEVICE_WARNING = 2,
    CHARGE_WARNING = 3,
    ABNORMAL_WARNING = 4,
    TASK_WARNING = 5
};

// struct to update the database
struct EventData 
{
    int event_status;
    std::string event_content;
    std::string event_serial_num;
};

struct DiagEventData 
{
    struct EventData data_item;
    uint64_t event_timestamp;
};

struct DataBaseCmdStru
{
    int cmd_type; // query and report last cycle recovered event command type is 0 , update this cycle event data to web directly command is 1
    struct DiagEventData event_data; // event data is here
};

class PostToWebContent{
    public:
        PostToWebContent(){
            this->event_title = "";
        }

        PostToWebContent(std::string title, Json::Value content){
            this->event_title = title;
            this->event_content = content;
        }

        PostToWebContent(const PostToWebContent & tmp){
            this->event_title = tmp.event_title;
            this->event_content = tmp.event_content;
        }

        std::string & get_event_title(){return event_title;};
        Json::Value & get_event_content(){return event_content;};

    private:
        std::string event_title;
        Json::Value event_content;
};

class Diagnostics {
public:
    virtual ~Diagnostics();
    static Diagnostics* get_instance();
    bool robot_collision_;
    Json::Value getValue() const;
    Json::Value diagnosticsRobotInfo(std::string id, int64_t timestamp);
    //bool getRobotBraked();
    int getBatLevel();
    std::string getCurrentUpgradeStatus();
    // public function to update the event database for other class to use
    int UpdateDiagEventDataBase(const DiagEventData & diag_event_data);
private:
    Diagnostics();
    std::string getMainVersion(void);
    int initEventVarStats(); // maybe unused
    void initDiagAttr(void);
    // data base process
    bool initDiagDataBase(void);
    bool initDiagEventStoreDataBase(void);
    void reinitDiagDataBase(void);
    void reinitDiagStoreDataBase(void);
    void processDiagEvent(void);
    int getAccordEventStats(const std::string & event_name, const int & status_this, int & status, std::string & serial_num);
    void reportRecoveredEvent(const DataBaseCmdStru & event_data_this, const int & event_status_last , const std::string & serial_number_last);
    void diagEventCmdAdd(const struct DataBaseCmdStru & event_msg);
    int getEventStatusAccordingToEventContent(const std::string & event_content, int & event_status);
    bool addEventStoreDataBase(const std::string & eventId , const Json::Value event_msg);
    int deleteRecordWithId(int id);
    void storeDiagEventThread(void);
    void reportItvEventThread(void);
    void postDelayedEvent(void);
    void waitNotEmptyDataBase(void);
    //int postToWebDelay(void);
    int checkCanSend(const std::string & title, const std::string & eventId, const Json::Value & js_str); // check if we are conected to remote mqtt , if it is ok , we can send the message
    void diagStoreEventAdd(const std::string title, const Json::Value event_msg);
    void handleSend(const boost::system::error_code& error);
    void checkDevStatus(void);
    //void diagDataThread();
    //void spinStart();
    void diagUdpSpin();
    void checkBatteryLevel(int level);
    void checkBatteryChasisOverTemp();
    void checkLevelSensorStatus();
    bool checkRobotBrake();
    bool checkRebootBrake();
    void checkRobotCollision();

    //void checkCollisionTrigEvent();
    //void checkBumperTriggered();
    //void checkEmergeButton();
    
    
    // check and report event
    void checkUltraErrorEvent();
    

    void checkGyroErrorEvent();
    void check4gErrorEvent();
    void checkFanErrorEvent();
    void checkRobotEmergeBrakeEvent();
    void checkGpsStats();
    //void checkBatteryEvent();
    int deterBatteryError(unsigned short low_byte, unsigned short mid_byte, unsigned short high_byte);
    void Get4gState(Json::Value & tmp);
    void checkSensorLiquidEvent();
    //void notifyTelecomEvent(int op);
    void notifyGpsStatus(int gps_status);
    void notifyBatteryEvent(int bat_level);
    void notifyUpgradeEvent(bool upgrade_stats);

    // notify warn event to web
    void notifyDeviceWarningEvent(const std::string & event_content , const int & status, const uint64_t & timestamp, const std::string & serial_number);
    void notifyAbnormalWarningEvent(const std::string & event_content , const int & status, const uint64_t & timestamp, const std::string & serial_number);
    void notifyBatteryWarningEvent(const std::string & event_content ,const uint64_t & timestamp, const std::string & serial_number);
    void checkBatteryWarningEvent(int32_t low, int32_t mid, int32_t hight);
    void checkBatteryStartup(const int32_t &status);
    void notifyDeviceWarningEventNew(const std::string & event_content , const int & status, const uint64_t & timestamp, const std::string & serial_number, const std::string type, const int32_t value);
    void checkDriverWarningEventStartup(const tinyros::atris_msgs::chassis_abnor_info &msg);
    void checkDriverLeftFrontWarningEvent(const tinyros::atris_msgs::chassis_abnor_info &msg);
    void checkDriverRightFrontWarningEvent(const tinyros::atris_msgs::chassis_abnor_info &msg);
    void checkDriverLeftRearWarningEvent(const tinyros::atris_msgs::chassis_abnor_info &msg);
    void checkDriverRightRearWarningEvent(const tinyros::atris_msgs::chassis_abnor_info &msg);
    void diagEventCmdAddCycle(const int32_t &cmd_type, const tinyros::Time &now, int32_t status, const std::string &event_content, const std::string & serial_num);
    void checkCollisionEvent(int32_t status);
    void checkUltraStartup(void);

    void reportBatteryLevelLowUrgent(void);
    void checkIfNeedReportStartUp(int bat_level);
    void reportBatteryLevelLowNormal(void);

    //void sendPTZErrorEventFake(int ptz_error_status);
    //void sendBmsErrorEventFake(int bms_error_status);
    //void sendChassisErrorEventFake(int chassis_error_status);
    void setPTZSerialNum(std::string ptz_serial){boost::unique_lock<boost::mutex> lock(ptz_serial_num_mutex_); ptz_serial_num_ = ptz_serial;};
    std::string getPTZSerialNum(void){boost::unique_lock<boost::mutex> lock(ptz_serial_num_mutex_); return ptz_serial_num_;};

    void setBmsSerialNum(std::string bms_serial){boost::unique_lock<boost::mutex> lock(main_bms_serial_num_mutex_); main_bms_serial_num_ = bms_serial;};
    std::string getBmsSerialNum(void){boost::unique_lock<boost::mutex> lock(main_bms_serial_num_mutex_); return main_bms_serial_num_;};

    void setChassisSerialNum(std::string chassis_serial){boost::unique_lock<boost::mutex> lock(chassis_serial_num_mutex_); chassis_serial_num_ = chassis_serial;};
    std::string getChassisSerialNum(void){boost::unique_lock<boost::mutex> lock(chassis_serial_num_mutex_); return chassis_serial_num_;};

    void setCCBSerialNum(std::string ccb_serial){boost::unique_lock<boost::mutex> lock(ccb_serial_num_mutex_); ccb_serial_num_ = ccb_serial;};
    std::string getCCBSerialNum(void){boost::unique_lock<boost::mutex> lock(ccb_serial_num_mutex_); return ccb_serial_num_;};

    void setSYSSerialNum(std::string sys_serial){boost::unique_lock<boost::mutex> lock(sys_serial_num_mutex_); sys_serial_num_ = sys_serial;};
    std::string getSYSSerialNum(void){boost::unique_lock<boost::mutex> lock(sys_serial_num_mutex_); return sys_serial_num_;};

    //void handleSend(const boost::system::error_code& error);
    void messageInstantReceive (const atris_msgs::SignalMessage& msg);
    void notifyRobotControlStatus(const atris_msgs::SignalMessage& msg);
    bool doGetDiagnosticsInfo(atris_msgs::GetDiagnosticsInfo::Request& req, atris_msgs::GetDiagnosticsInfo::Response& res);
    bool doGetOtaPreCondition(atris_msgs::GetOTAPreCondition::Request& req, atris_msgs::GetOTAPreCondition::Response& res);
    void requestDiagnosticsRobotInfo(const atris_msgs::SignalMessage& msg, const Json::Value& root);
    void imuInfoTopicCb (const atris_msgs::RobotImu& msg);
    void robotConnectStateCb(const atris_msgs::NetworkState &msg);
    void on_receive_odom(const tinyros::atris_msgs::RobotPose& msg);
    void on_recv_chassis_diag_info(const tinyros::atris_msgs::chassis_abnor_info &msg);
    //void getOdomInfo(Json::Value & tmp);
    //void getImuError(Json::Value & tmp);
    void robotInfoTopicCb (const atris_msgs::RobotInfo& msg);
    void notifyRobotInfoToShadow(void);
    void notifyPtzStatusToShadow(void);
    void notifyStandbyModeToShadow(void);
    void upload_logs(const atris_msgs::SignalMessage &msg);
    void on_recv_upgrade_stats(const atris_msgs::UpgradeStatus& msg);
    void on_recv_mcu_upload_log_status(const atris_msgs::McuLogCtrl& msg);
    long long __getmstime(void);
    int getSysUpTime(float & running_hours);
    int createMcuUploadDir(void);
    bool createMultiLevelDir(const char *path);
    int reqChassisLogs(void);
    int reqMonitorLogs(void);
    void sendMcuUploadLogReq(const std::string & board_name);
    int waitChassisUploadLogFinish(void);
    int waitMonitorUploadLogFinish(void);

    void checkCCBWhenStartUp(int ccb_error_status);
    void checkSYSWhenStartUp(int sys_error_status);
    void checkLidarWhenStartUp(int lidar_error_status);
    void checkLidarErrorEvent(int lidar_error_status);
    void checkEmergeButtonWhenStartUp(int button_status);
    void checkEmergeButtonEvent(int button_status);
    void checkBMSWhenStartUp(int bms_error_status);
    void checkBMSErrorEvent(int bms_error_status);
    void checkPTZWhenStartUp(int ptz_error_status);
    void checkPTZErrorEvent(int ptz_error_status);
    void checkChassisWhenStartUp(int chassis_error_status);
    void checkChassisErrorEvent(int chassis_error_status);
    void checkGpsError(void);
    void checkImuError(void);
    void UpDateOdomInfo(void);
    void updateGpsRecvTime(void);

    // voip status part
    inline void setVoipCallingStatus(int stats){boost::unique_lock<boost::mutex> lock(voip_status_mutex_); voip_calling_status_ = stats;};
    inline int getVopCallingStatus(){boost::unique_lock<boost::mutex> lock(voip_status_mutex_); return voip_calling_status_;};
    void requestPtzParam(void);
    void run_mode_thread(void);
    int determineCurrentRunMode(void);
    void notifyCurrentRunMode(const int & current_robot_mode);
    void robotRunModeInfoCb(const atris_msgs::RobotRunMode &msg);
    int getRunModeTransCode(int & run_mode);
    std::string getRunModeStr(int mode);
    void setRobotRunMode(uint8_t robot_mode, uint8_t status);

    void sysDiagDelayProc(void); // system self diagnostics
    
    // itv event store num operation
    void eventStoreNumInc(void);
    void eventStoreNumDec(void);
    int getEventStoreNum(void);
    void setEventStoreNum(int num);

    // event check
    void checkLidarDev(void);
    void checkPtzDev(void);
    void checkBmsDev(void);
    void checkChassisDriver(void);
    void checkCCBBoard(void);
    void checkSYSBoard(void);
    void checkCCBErrorEvent(int ccb_error_status);
    void checkSYSErrorEvent(int sys_error_status);

    int getConnectState(void); // get mqtt connect state
    void sendDeviceWarning(int on_off);

    float floatTrim(float val);
    double doubleTrim(double val);
private:
    static boost::mutex mutex_;
    static Diagnostics* diagnostics_;
    boost::asio::io_service io_service_;
    CTimer * pDiagSpinTimer;
    //CTimer * pPostToWebDelayTimer;
    CTimer * pSelfDiagTimer;
    Utils *utils;
    Json::Value diag_;
    bool data_base_init_ok_;
    bool event_data_base_init_ok_;
    // post delayed event to web in case the signaling process is not up
    bool wait_can_send_;
    MsgList<PostToWebContent> event_store_list_;

    boost::mutex wait_can_send_mutex_;
    boost::mutex event_list_mutex_;
    boost::condition_variable event_list_cond_;
    MsgList<DataBaseCmdStru> event_msg_list_;

    double battery_check_time_last_;
    bool isUpgrading;
    boost::thread* log_thread_;
    bool log_uploading_;
    bool middle_fan_enable_;
    int level_trig_once_flg_;
    int battery_level_last_;
    int bumper_state_last_;
    int button_state_last_;
    int system_check_cnt_;
    int battery_temp_last_;
    // odom info
    odom_info odom_topic_info;

    // mcu upload log part
    bool chassis_upload_log_finish_;
    bool monitor_upload_log_finish_;
    boost::mutex chassis_upload_log_mutex_;
    boost::condition_variable chassis_upload_log_cv_;
    boost::mutex monitor_upload_log_mutex_;
    boost::condition_variable monitor_upload_log_cv_;

    // voip status part
    boost::mutex voip_status_mutex_;
    int voip_calling_status_;
    long long diag_spin_called_time_;

    boost::mutex diag_database_mutex_; // diag event store database mutex
    boost::mutex store_num_mutex_; // diag event store database mutex
    int event_store_num_;

    double battery_temp_check_time_last_; // battery overtemp check last
    long long imu_recv_time_last_;
    int imu_error_;
    long long odom_recv_time_last_;
    long long gps_recv_time_last_;
    long long bms_info_recv_last_;
    int bms_warning_stats_;
    int chassis_warning_stats_;
    int odom_error_;
    long long chassis_lf_recv_last_;
    long long chassis_rf_recv_last_;
    long long chassis_lr_recv_last_;
    long long chassis_rr_recv_last_;
    int chassis_lf_error_stats_;
    int chassis_rf_error_stats_;
    int chassis_lr_error_stats_;
    int chassis_rr_error_stats_;
    int ccb_error_stats_;
    int sys_error_stats_;
    int simulation_event_switch_;

    // robot run mode
    boost::mutex run_mode_mutex_;
    unsigned long  sys_tick_;
    bool mode_priority_thread_exit_;
    uint8_t run_mode_status_[NUMBER_OF_MODE_PRIORITY]; // store if the mode get the control of robot
    int robot_run_mode_;

    // diag database event status variable list
    int localization_lost_stats_;
    int signal_bad_stats_;
    int ptz_error_stats_;
    int lidar_error_stats_;
    int ultra_error_stats_;
    int ads_error_stats_;
    int imu_error_stats_;
    int chassis_error_stats_;
    int battery_error_stats_;
    int speaker_error_stats_;
    int mic_error_stats_;
    int voip_error_stats_;
    int flood_warning_stats_;
    int overtemp_warning_stats_;
    int emerge_stop_warning_stats_;
    int collision_warning_stats_;
    int abnormal_shutdown_warning_stats_;
    // system check condition part
    bool system_delay_check_;
    int bms_communication_ok_;
    bool isBmsCharging_; // not do device check if robot is charging
    int charging_status_last_;
    int machine_status_;
    //int low_battery_warn_level_;
    boost::mutex ptz_serial_num_mutex_;
    std::string ptz_serial_num_;
    boost::mutex main_bms_serial_num_mutex_;
    std::string main_bms_serial_num_;
    boost::mutex chassis_serial_num_mutex_;
    std::string chassis_serial_num_;
    boost::mutex ccb_serial_num_mutex_;
    std::string ccb_serial_num_;
    boost::mutex sys_serial_num_mutex_;
    std::string sys_serial_num_;

    // mqtt connect status , we need temporarily store event when we are disconnected from remote mqtt
    int mqtt_connect_state_;
    boost::mutex connect_state_mutex_;
    boost::condition_variable connect_state_cond_;
    boost::mutex database_wait_mutex_;
    boost::condition_variable database_wait_mutex_cv_;


    ros::NodeHandle nh_;
    ros::Publisher shttpd_diag_pub_;
    ros::Publisher aisound_tts_pub_;
    ros::Publisher robot_braked_pub_;
    ros::Publisher reboot_braked_pub_;
    ros::Publisher protect_mode_pub_;
    ros::Publisher power_off_pub_;
    ros::Publisher send_can_cmd_pub_;
    ros::Publisher mqtt_signal_pub_;
    ros::Publisher recharge_battery_pub_;
    ros::Publisher middle_fan_ctrl_pub_;
    ros::Publisher mcu_upload_log_req_pub_;
    ros::Subscriber signal_req_sub_;
    ros::Subscriber signal_resp_sub_;
    ros::Subscriber diag_info_sub_;
    ros::Subscriber upgrade_status_sub_;
    ros::Subscriber mcu_upload_log_resp_sub_;
    ros::Subscriber imu_info_sub_;
    ros::Subscriber robot_run_mode_sub_;
    ros::Subscriber mqtt_connect_state_sub_;
    tinyros::Subscriber<tinyros::atris_msgs::chassis_abnor_info, Diagnostics> chassis_diag_sub_;
    tinyros::Subscriber<tinyros::atris_msgs::RobotPose, Diagnostics> robot_pose_sub_;
    ros::ServiceClient get_lamp_status_srv_client_;
    ros::ServiceClient get_standby_mode_srv_client_;
    ros::ServiceClient get_ppplaying_status_srv_client_;
    ros::ServiceClient get_disperse_status_srv_client_;
    ros::ServiceClient get_patrol_status_srv_client_;
    ros::ServiceClient get_ptz_status_srv_client_;
    ros::ServiceClient peripheral_ctrl_client_;
    ros::ServiceClient get_sw_version_srv_client_;
    ros::ServiceClient get_chassis_info_srv_client_;
    ros::ServiceServer get_ota_pre_condition_srv_;
    ros::ServiceServer diag_info_srv_;

};

#endif /* DIAGNOSTICS_H_ */