#ifndef MCU_MANAGER_H__
#define MCU_MANAGER_H__

#include <ros/ros.h>
#include "tiny_ros/ros.h"
#include "log/log.h"
#include "tiny_ros/atris_msgs/CanPkg.h"
#include "atris_msgs/RobotInfo.h"
#include "atris_msgs/SignalMessage.h"
#include "imemory/atris_imemory_api.h"
#include "atris_msgs/PowerChargeCmd.h"
#include "atris_msgs/GetSwVersion.h"
#include "atris_msgs/GetStandbyMode.h"
#include "atris_msgs/McuLogCtrl.h"
#include "atris_msgs/RobotRunMode.h"
#include "atris_msgs/PeripheralControl.h"
#include "atris_msgs/AntiDrop.h"
#include "atris_msgs/AisoundTTS.h"
#include "tts_strings/tts_strings.h"
#include <json/json.h>

#define MCU_REPORT_MSG_DBG 1
#define ULTRA_NUM 4

#define REMOTE_CTRL_SBUS_CENTER         1000
#define REMOTE_CTRL_SBUS_MAX_VALUE      800
#define REMOTE_CTRL_ABNOR_VALUL         100
#define REMOTE_KNOB_MIN_VALUE           306
#define REMOTE_KNOB_CENTER_VALUE        1000
#define REMOTE_KNOB_MAX_VALUE           1694
#define REMOTE_KNOB_CTRL_RANGE          REMOTE_KNOB_MAX_VALUE-REMOTE_KNOB_MIN_VALUE
#define KNOB_NOISE_TOLERANCE_MAX        2
#define KNOB_WAIT_STABLE_CNT            5 // 5 * 50ms = 250ms
#define KNOB_STABLE_TOLERANCE_INTERVAL  5
#define PTZ_CONTROL_VERTICAL_FACTOR     7.7 // (1694 - 306)/ 180
#define PTZ_CONTROL_HORIZONTAL_FACTOR   3.8
// the value is clapped between 0 and 180
// and a offset of 90 is substract from this value

#define DF_REMOTE_OFF_LINE   -1
#define DF_REMOTE_ON    0
#define DF_REMOTE_ON_LINE    1
#define DF_REMOTE_CONTROL   2
#define DF_REMOTE_ROCKER_NOT_ON_CENTER   3

// remote control ptz move
typedef enum __PTZ_CTRL_UP_DOWN_STATE
{
    PTZ_VERTICAL_MOVE_UP = 0,
    PTZ_VERTICAL_MOVE_DOWN = 1,
    PTZ_VERTICAL_MOVE_IDLE = 2
} PTZ_CTRL_UP_DOWN_STATE;

typedef enum __PTZ_CTRL_LEFT_RIGHT_STATE
{
    PTZ_HORIZONTAL_MOVE_LEFT = 0,
    PTZ_HORIZONTAL_MOVE_RIGHT = 1,
    PTZ_HORIZONTAL_MOVE_IDLE = 2
} PTZ_CTRL_LEFT_RIGHT_STATE;

typedef enum __PTZ_CTRL_BRUSH_STATE
{
    PTZ_BRUSH_STATE_OFF = 0,
    PTZ_BRUSH_STATE_ON = 1
} PTZ_CTRL_BRUSH_STATE;

typedef enum __REMOTE_CTRL_BRAKE_STATE
{
    REMOTE_ENABLE_BRAKE_NULL = 0,
    REMOTE_ENABLE_BRAKE_OFF = 1
} REMOTE_CTRL_BRAKE_STATE;

typedef enum __REMOTE_CTRL_SUPPLEMENT_LIGHT_STATE
{
    PTZ_SUPPLEMENT_LIGHT_OFF = 0,
    PTZ_SUPPLEMENT_LIGHT_ON = 1
} REMOTE_CTRL_SUPPLEMENT_LIGHT_STATE;

typedef enum __MCU_CTRL_REQ_ERR_CODE
{
	CTRL_REQ_SUCCESS = 0,
    CTRL_REQ_DUPLICATE = -1,
    CTRL_REQ_TIMEOUT = -2,
    CTRL_REQ_FAILED = -3,
    CTRL_REQ_PARAM_INVALID = -4,
    CTRL_REQ_INNER_ERROR = -5,
    CTRL_REQ_QUERY_FAILED = -6
} MCU_CTRL_REQ_ERR_CODE;

typedef enum __INDICATOR_LIGHT_STYLE
{
    STABLE = 0,
    BREATHE = 1,
    FLASHING = 2
} INDICATOR_LIGHT_STYLE;

typedef enum __INDICATOR_LIGHT_COLOR
{
    RED = 1,
    ORANGE = 2,
    GREEN = 3,
    CYAN = 4,
    BLUE = 5,
    PURPLE = 6,
    WHITE = 7
} INDICATOR_LIGHT_COLOR;

typedef enum __SYS_MONITOR_CMD_SET
{
    INDICATOR_LIGHT_REQ = 201,
    INDICATOR_LIGHT_RESP = 202,

    CONTROLLED_SOURCE_REQ = 211,
    CONTROLLED_SOURCE_RESP = 212,

    VOLTAGE_MONITOR_INFO = 214,
    CURRENT_MONITOR_INFO = 216,

    QUERY_ULTRA_VERSION_REQ = 221,
    QUERY_ULTRA_VERSION_RESP = 222,

    ULTRA_SOUND_MONITOR_INFO = 224,
    TEMP_HUMI_MONITOR_INFO = 226,
    ANTI_DROP_MONITOR_INFO = 228,

    FAN_CONTROL_REQ = 241,
    FAN_CONTROL_RESP = 242,

    QUERY_MONITOR_VER_REQ = 291,
    QUERY_MONITOR_VER_RESP = 292,

    MONITOR_LOG_FILE_GET_REQ = 293,
    MONITOR_LOG_FILE_GET_RESP = 294

} SYS_MONITOR_CMD_SET;

typedef enum __CHASSIS_CONTROLLER_CMD_SET
{
	// battery data
    BATTERY_INFO_REPORT = 100,

    // query battery log cmd
    QUERY_BATTERY_LOG_CMD_REQ = 101,
    QUERY_BATTERY_LOG_CMD_RESP = 102,

    // mcu notify open or shutdown event operation
    CHASSIS_CONTROLLER_NOTIFY_ROBOT_OP_REQ = 112,
    CHASSIS_CONTROLLER_NOTIFY_ROBOT_OP_RESP = 111, // just return the same info back to chassis controller

    // robot inform mcu that we want to do shutdown or open operation
    ROBOT_INFORM_CHASSIS_CONTROLLER_OP_REQ = 113,
    ROBOT_INFORM_CHASSIS_CONTROLLER_OP_RESP = 114,

    // robt inform mcu to do the recharge request, now we have only leave pile command
    ROBOT_TO_CHASSIS_RECHARGE_REQ = 115,
    ROBOT_TO_CHASSIS_RECHARGE_RESP = 116,

    // mcu to robot recharge status notification
    CHASSIS_TO_ROBOT_RECHARGE_RESULT_REQ = 118,
    CHASSIS_TO_ROBOT_RECHARGE_RESULT_RESP = 117,

    // robot notify to mcu that it is stop moving and ok to start charging
    ROBOT_TO_CHASSIS_RECHARGE_IN_POSITION_REQ = 119,
    ROBOT_TO_CHASSIS_RECHARGE_IN_POSITION_RESP = 120,

    CHASSIS_TO_ROBOT_NOTIFY_MACHINE_STATUS = 124, // whether or not the machine is in standby mode

    // query and set chassis controller controlled source request and response
    CHASSIS_CONTROLLED_SOURCE_REQ = 131,
    CHASSIS_CONTROLLED_SOURCE_RESP = 132,

    // chassis notify voltage info report
    CHASSIS_REPORT_VOLTAGE_INFO = 134,

    // chassis notify current info report
    CHASSIS_REPORT_CURRENT_INFO = 136,

    // remote controller info report
    CHASSIS_REPORT_REMOTE_CONTROLLER_INFO = 142,

    // chassis report anti collision info
    CHASSIS_REPORT_ANTI_COLLISION_INFO = 144,

    // chassis report temperature and humidity info to robot
    CHASSIS_REPORT_TEMP_HUMI_INFO = 146,
    
    // request to chassis software emergency stop
    ROBOT_TO_CHASSIS_EMERGE_STOP_REQ = 161,

    // chassis response to robot emergency stop result
    CHASSIS_TO_ROBOT_EMERGE_STOP_RESP = 162,

    // chassis report stop reason
    CHASSIS_REPORT_STOP_REASON = 164,

    // query chassis mcu version info request
    QUERY_CHASSIS_VER_REQ = 191,

    // query chassis mcu version info response
    QUERY_CHASSIS_VER_RESP = 192,

    // query bms hardware version request
    QUERY_BMS_HW_VER_REQ = 103,

    // query bms hardware version response
    QUERY_BMS_HW_VER_RESP = 104,
    
    // query bms software version request
    QUERY_BMS_SW_VER_REQ = 105,

    // query bms software version response
    QUERY_BMS_SW_VER_RESP = 106,

    // robot to chassis set air transport mode request
    ROBOT_TO_CHASSIS_SET_TRANSPORT_MODE_REQ = 165,

    // chassis to robot set air transport mode response
    CHASSIS_TO_ROBOT_SET_TRANSPORT_MODE_RESP = 166,

    // request chassis to upload file
    CHASSIS_LOG_FILE_GET_REQ = 193,
    
    // response of chassis to upload file
    CHASSIS_LOG_FILE_GET_RESP = 194


} CHASSIS_CONTROLLER_CMD_SET;

struct SysMonResp {
  std::mutex mutex;
  tinyros::atris_msgs::CanPkg resp;
  std::shared_ptr<std::condition_variable> cond = nullptr;
};

typedef std::shared_ptr<SysMonResp> SysMonRespPtr;
typedef std::shared_ptr<std::condition_variable> SysMonRespCondPtr;

struct ChassisCtrlResp {
  std::mutex mutex;
  tinyros::atris_msgs::CanPkg resp;
  std::shared_ptr<std::condition_variable> cond = nullptr;
};

typedef std::shared_ptr<ChassisCtrlResp> ChassisCtrlRespPtr;
typedef std::shared_ptr<std::condition_variable> ChassisCtrlRespCondPtr;

class McuManager
{
public:
    ~McuManager() {}
    // get instance
    static McuManager *get_instance(){
        static McuManager instance;
        return &instance;
    }

    // indicator light control
    int queryIndicatorLightState(int & style, int & color, int & brightness); // query the state of the indicator light
    int setIndicatorLightStyle(int light_style);
    int setIndicatorLightColor(int light_color);
    int setIndicatorLightBrightness(int light_brightness);
    int releaseIndicatorLightControl(void);

    // controlled source voltage control
    int querySysMonitorControlSourceState(int & control_source_1, int & control_source_2);
    int setSysMonitorControlSource1(int status);
    int setSysMonitorControlSource2(int status);
 
    int doRobotOpReq(int op); // start shutdown or reboot the machine operation command
    int startRobotRechargeOperation(int op); // start recharge operation
    // chassis voltage source control
    int queryChassisControlSourceState(int & control_source_1, int & control_source_2);
    int setChassisControlSource1(int status);
    int setChassisControlSource2(int status);

    // fan control
    int setSysMonitorFanSpeed(int speed);
    int querySysMonitorFanStatus(int & fan_speed, int & fan1_error_status , int & fan2_error_status);

    int queryBatteryLog(char * pcBuf);
    int init();
    void on_recv_peripheral_ctrl(const atris_msgs::SignalMessage &msg);
    void on_recv_navigation_charge_request(const atris_msgs::PowerChargeCmd &msg);
    void on_recv_mcu_upload_log_request(const atris_msgs::McuLogCtrl& msg);
    bool on_recv_peripheral_device_ctrl(atris_msgs::PeripheralControl::Request& req, atris_msgs::PeripheralControl::Response& res);

    int setChassisSoftEmergeStop(int status);

private:
    McuManager();
    long long getsysmstime(void);

    std::map<int, SysMonRespPtr> sys_monitor_resp_map_;
    std::mutex sys_monitor_map_mutex_;
    std::map<int, ChassisCtrlRespPtr> chassis_controller_resp_map_;
    std::mutex chassis_controller_map_mutex_;
    long long remote_control_time_recv_last_;

    void on_recv_sys_monitor_resp(const tinyros::atris_msgs::CanPkg &msg);
    void on_recv_chassis_controller_message(const tinyros::atris_msgs::CanPkg &msg);
    void reqGetMcuLog(const std::string & board_name);

    void processVoltageInfoMsg(const tinyros::atris_msgs::CanPkg &msg);
    void processCurrentInfoMsg(const tinyros::atris_msgs::CanPkg &msg);
    void processUltraMsg(const tinyros::atris_msgs::CanPkg &msg);
    void processTempHumiMsg(const tinyros::atris_msgs::CanPkg &msg);
    void processAntiDropMsg(const tinyros::atris_msgs::CanPkg &msg);
    void processMonitorUploadLogResp(const tinyros::atris_msgs::CanPkg &msg);
    bool removeSysMonDupRequest(int msg_id);
    // query cmd request, for internal use
    int queryIndicatorLight(int & style, int & color, int & brightness);
    int querySupplementLight(int & light_status, int & light_brightness);
    int querySysMonitorControlSource(int & control_source_1, int & control_source_2);
    // query ultra sound version info
    int queryUltraSoundVersion(int & version_info);
    int do_get_software_version_task(void);
    void doProcSysMonitorCmdResp(const tinyros::atris_msgs::CanPkg & msg);

    void printCanPkgMsg(const tinyros::atris_msgs::CanPkg &msg);
    void procBatMonInfo(const tinyros::atris_msgs::CanPkg &msg);
    void procChassisVoltageMsg(const tinyros::atris_msgs::CanPkg &msg);
    void procChassisCurrentMsg(const tinyros::atris_msgs::CanPkg &msg);
    void procChassisMachineStatus(const tinyros::atris_msgs::CanPkg &msg);
    void procChassisRemoteControlMsg(const tinyros::atris_msgs::CanPkg &msg);
    void doChassisControllerReqResp(const tinyros::atris_msgs::CanPkg &msg);

    int chassisNotifyRobotOperation(const tinyros::atris_msgs::CanPkg &msg); // chassis notify operation to robot side
    int chassisNotifyRechargeResult(const tinyros::atris_msgs::CanPkg &msg); // chassis notify charging result to robot
    void chassisNotifyOperationResp(const tinyros::atris_msgs::CanPkg &msg);
    int notifyRobotInPosition(int charge_result); // notify to chassis controller that robot in position
    bool removeChassisDupRequest(int msg_id);
    int queryChassisControlSource(int & control_source_1, int & control_source_2);
    void procAntiCollisionInfo(const tinyros::atris_msgs::CanPkg &msg);
    void procTempHumiInfo(const tinyros::atris_msgs::CanPkg &msg);
    void procChassisStopReasonInfo(const tinyros::atris_msgs::CanPkg &msg);
    void processMonitorMcuVersion(const tinyros::atris_msgs::CanPkg &msg);
    void procChassisMcuVersion(const tinyros::atris_msgs::CanPkg &msg);
    void procBMSHwVer(const tinyros::atris_msgs::CanPkg &msg);
    void procBMSSwVer(const tinyros::atris_msgs::CanPkg &msg);
    void procChassisUploadLogResp(const tinyros::atris_msgs::CanPkg &msg);
    void procChassisSetTransportModeResp(const tinyros::atris_msgs::CanPkg &msg);

    // send message to shadow server
    void notifyRemoteControllerInControl(uint8_t ctrl_switch);

    void reqCtrlPtzMove(int direction);
    void remoteControlPtzUpDown(int chan_val);
    void remoteControlPtzLeftRight(int chan_val);
    void remoteControlPtzHorizontal(int chan_val);
    void remoteControlPtzVertical(int chan_val);
    void reqCtrlPtzMoveDirection(int direction);
    void reqCtrlPtzMoveStop(void);
    // ptz brush part
    void remoteCtrlPtzBrush(int chan_val); // deal with channel value
    void reqCtrlPtzBrush(int switch_val);
    // remote ctrl chassis unlock
    void remoteCtrlChassisUnlock(int chan_val);
    int sendChassisUnlockNonBlock(int status);
    // remote ctrl ptz supplement light
    void remoteCtrlPtzLight(int chan_val);
    void reqCtrlPtzLight(int switch_val);
    // remote control set chassis in transport mode or unset transport mode
    void remoteControlChassisTransportMode(int chan_val);

    int filterVrANoise(int chan_val);
    int filterVrBNoise(int chan_val);

    int & getVraVal(){return vra_value_last_;};
    void setVraVal(int val){vra_value_last_ = val;};

    int & getVrbVal(){return vrb_value_last_;};
    void setVrbVal(int val){vrb_value_last_ = val;};
    int chanValClamp(int val, int min, int max);
    bool waitInputVraStable(int channel_value);
    bool waitInputVrbStable(int channel_value);
    void reqCtrlPtzMoveVerticalAngle(int angle);
    void reqCtrlPtzMoveHorizontalAngle(int angle);

    int setChassisTransportMode(int status);
    
    // debug helper functions
    std::string get_light_style_str(int style_id);
    std::string get_light_color_str(int color_id);
    
    // upgrade software version part
    std::string getICPUpgradeStatus(void);
    void queryMonitorMcuVersionInfo(void);
    void queryChassisMcuVersionInfo(void);
    void queryBMSHardWareVersionInfo(void);
    void queryBMSSoftWareVersionInfo(void);

    bool doGetSwVersion(atris_msgs::GetSwVersion::Request& req, atris_msgs::GetSwVersion::Response& res);
    bool doGetRobotStandbyStatus(atris_msgs::GetStandbyMode::Request& req, atris_msgs::GetStandbyMode::Response& res);
    std::string getImxVersion();

    std::string imx_version_;
    std::string chassis_sw_version_;
    int chassis_base_hw_version_;
    int chassis_core_hw_version_;
    std::string monitor_sw_version_;
    int monitor_base_hw_version_;
    int monitor_core_hw_version_;
    std::string bms_hw_version_;
    std::string bms_sw_version_;
    volatile bool isMonitorMcuVerReceived;
    volatile bool isChassisMcuVerReceived;
    volatile bool isBMSSwVerReceived;
    volatile bool isBMSHwVerReceived;

    // robot standby mode
    
    // shutdown process
    void procNormalPowerOff(void);
    void proc_poweroff_thread(void);
    int send_poweroff_finish(void);
    void procChassisChargerPlugged(void);
    void proc_charger_plugged_thread(void);
    int send_charger_plugged_proc_finish(void);
    void procChassisRechargeStandby(void);
    void proc_recharge_standby_thread(void);
    int send_recharge_proc_finish(void);
    // set robot in standby mode or wake up
    int reqRobotInStandbyMode(void);
    int reqRobotStandbyWakeup(void);

    void checkDeviceError(void);
    void sendAdsErrorStatus(int val);

    bool is_power_off_;
    bool is_charger_standby_;
    bool is_recharge_standby_;
    std::mutex power_off_mutex_;
    std::condition_variable power_off_cv_;
    std::mutex charger_standby_mutex_;
    std::condition_variable charger_standby_cv_;
    std::mutex recharge_standby_mutex_;
    std::condition_variable recharge_standby_cv_;

    // indicator light control
    std::mutex indicator_light_mutex_;
    int indicator_light_style_;
    int indicator_light_color_;
    int indicator_light_brightness_;

    // control source for system monitor
    std::mutex sys_controlled_source_mutex_;
    int sys_controlled_source1_;
    int sys_controlled_source2_;

    // fan control for system monitor

    std::mutex sys_fan_control_mutex_;
    int sys_fan_speed_;
    int sys_fan1_error_status_;
    int sys_fan2_error_status_;

    // ultra sound version for system monitor
    int ultra_version_info_;
    int diag_status;
    long long ultra_info_msg_recv_last_;
    uint16_t ultra_data[ULTRA_NUM];

    // chassis controlled voltage source
    std::mutex chassis_controlled_source_mutex_;
    int chassis_controlled_source1_;
    int chassis_controlled_source2_;

    int remote_status_;
    int ch_flag_;
    int brake_state_;
    int supplement_light_state_;

    // class private info
    std::string rbt_sn_;
    int emergency_button_status_;
    int collision_bumper_status_;
    int brake_enable_status_;

    int anti_drop_status_;
    long long recv_anti_drop_last_;
    long long recv_ads_last_;

    // tinyros
    tinyros::Subscriber<tinyros::atris_msgs::CanPkg, McuManager> chassis_controller_report_sub_; // receive chassis monitor information
    tinyros::Publisher chassis_controller_service_pub_; // publish message to chassis monitor
    tinyros::Publisher sys_monitor_service_pub_;
    tinyros::Subscriber<tinyros::atris_msgs::CanPkg, McuManager> sys_monitor_resp_sub_;
    
    // standard ros
    ros::NodeHandle nh_;
    ros::Publisher ptz_ctrl_pub_;
    ros::Subscriber signal_req_sub_;
    ros::Publisher diag_info_pub_;
    ros::Publisher anti_drop_pub_;
    ros::ServiceServer get_sw_version_srv_;
    ros::ServiceServer get_robot_standby_mode_srv;
    ros::ServiceServer peripheral_control_srv_;
    ros::Publisher aisound_tts_pub_;

    // charge command request and response
    ros::Publisher charge_response_pub_; // publish out charge response command from mcu
    ros::Subscriber charge_request_sub_; // receive charge command from navigation module
    ros::Publisher robot_run_mode_pub_; // publish remote control mode from remote controller status
    
    // mcu log upload request and response
    ros::Publisher mcu_upload_response_pub_; // publish the mcu upload result finish message to diagnostics module
    ros::Subscriber mcu_upload_request_sub_; // recv upload mcu log command from diagnostics module

    // ptz control state
    int ptz_up_down_state_;
    int ptz_left_right_state_;
    int ptz_brush_state_;
    int vra_value_last_;
    int vrb_value_last_;
    int vra_stable_count_;
    int vrb_stable_count_;
    int vra_stable_send_once_;
    int vrb_stable_send_once_;
    bool vra_stable_state_;
    int vra_filter_out_val_;
    int vrb_filter_out_val_;
    bool vrb_stable_state_;

    int swh_stable_send_once_;
    ros::Time swh_stable_time_last_;
    bool swh_start_to_count_;

    int robot_mode_; // normal = 0 , standby = 1
    std::mutex standby_mode_mutex_;
    std::condition_variable standby_mode_cv_;
    std::mutex wakeup_mutex_;
    std::condition_variable wakeup_cv_;

};

#endif
