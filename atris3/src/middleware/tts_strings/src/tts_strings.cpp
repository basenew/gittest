#include "tts_strings.h"
#include "imemory/atris_imemory_api.h"
#include "log/log.h"

static std::map<int, std::vector<std::string> > strings_map = {
    {TTSStrings::TTS_KEY_BACK, {"后退，请注意", "Attention, back!"}},
    {TTSStrings::TTS_KEY_TURN_LEFT, {"左转，请注意", "Turn left，be careful"}},
    {TTSStrings::TTS_KEY_TURN_RIGHT, {"右转，请注意", "Turn right，be careful"}},
    {TTSStrings::TTS_KEY_LOW_BATTERY, {"机器电量低，请充电", "Robot battery is low, please charge!"}},
    {TTSStrings::TTS_KEY_SWU_FINISHED, {"固件升级成功", "Firmware Upgrade succeeded!"}},
    {TTSStrings::TTS_KEY_SWU_INFO, {"固件升级中，请不要关机", "Firmware is upgrading, please don't shut down"}},
    {TTSStrings::TTS_KEY_SWU_FAIL_FIRMWARE, {"固件升级失败", "Firmware upgrade failed!"}},
    {TTSStrings::TTS_KEY_COLLISION, {"机器发生碰撞，请立即检查", "Robot was hit, please check immediately"}},
    {TTSStrings::TTS_KEY_CHASSIS_UNLOCKING, {"机器底盘解锁，请注意", "Attention, the robot chassis is unlocked!"}},
    {TTSStrings::TTS_KEY_CHASSIS_LOCK, {"底盘，锁定", "Chassis, Lock"}},
    {TTSStrings::TTS_KEY_TELECONTROL_POW_ON, {"摇控器连接成功", "Remote-control connected"}},
    {TTSStrings::TTS_KEY_TELECONTROL_POW_OFF, {"遥控器连接断开", "Remote-control disconnected"}},
    {TTSStrings::TTS_KEY_TELECONTROL_UNLOCK, {"摇控器解锁，请注意", "Attention, remote-control is unlocked!"}},
    {TTSStrings::TTS_KEY_TELECONTROL_LOCKING, {"摇控器锁定", "Remote-control is locked!"}},
    {TTSStrings::TTS_KEY_PLEASE_UNLOCK, {",请解锁", ",Please unlock"}},
    {TTSStrings::TTS_KEY_REMOTE_UNAVAILABLE, {"摇控不可用，请切换", "The remote control is unavailable. Please switch"}},
    {TTSStrings::TTS_KEY_ROCKER_BACK, {"摇杆回正，再解锁", "Rocker back up and unlock again"}},
    {TTSStrings::TTS_KEY_SAFE_MODE, {"安全模式", "safe mode"}},
    {TTSStrings::TTS_KEY_STD_MODE, {"标准模式", "Standard mode"}},
    {TTSStrings::TTS_KEY_EMC_MODE, {"紧急模式", "Emergency mode"}},
    {TTSStrings::TTS_KEY_PRESS_EMERGENCY, {"紧急制动生效", "Emergency brake is on !"}},
    {TTSStrings::TTS_KEY_RELEASE_EMERGENCY, {"紧急制动解除", "Emergency brake is off !"}},
    {TTSStrings::TTS_KEY_PWM_CHASSIS_LOCK, {"PWM底盘，锁定,三十秒解除", "Chassis, Lock"}},
    {TTSStrings::TTS_KEY_PWM_CHASSIS_UNLOCKING, {"PWM底盘解锁，请注意", "Chassis unlocking,please note"}},
    {TTSStrings::TTS_KEY_HIGH_BATTERY_TEMP_WARN, {"机器人电池温度过高，请关机", "High battery temperature, please power off the system"}},
    {TTSStrings::TTS_KEY_HIGH_BATTERY_TEMP_FATAL, {"机器人已到达保护温度，即将关机", "Robot battery temperature high, power off the system now"}},
    {TTSStrings::TTS_KEY_ENTER_PROTECT_MODE,{"机器人电池温度高，将减速行驶","Battery temperature is high, robot will slow down!"}},
    {TTSStrings::TTS_KEY_LEVEL_SENSOR_TRIG,{"机器进水，请及时排水并检查!","Robot is flooded, please drain off and check up"}},
    {TTSStrings::TTS_KEY_SYSTEM_SELF_CHECK_NORMAL,{"机器自检结束，系统运行正常","Robot activated , system function normal!"}},
    {TTSStrings::TTS_KEY_SYSTEM_SELF_CHECK_ABNORMAL,{"机器自检结束，系统运行异常","Robot activated, sytem function error!"}},
    {TTSStrings::TTS_KEY_HIGH_CALL_VOIP_NO_BOUND, {"机器人未绑定，无法接通", "Robot is not binded, can not connect the subscriber!"}},
    {TTSStrings::TTS_KEY_HIGH_CALL_VOIP_TIMEOUT, {"呼叫超时，请重试", "the subscriber can not be connected, please try again!"}},
    {TTSStrings::TTS_KEY_HIGH_CALL_VOIP_BUSY, {"正在通话中，请稍候再拨", "the subscriber is busy now, please try again!"}},
    {TTSStrings::TTS_KEY_DOCK_BEGAIN, {"机器人正在上桩，请注意", "Attention please, Robot is docking charging pile!"}},
    {TTSStrings::TTS_KEY_DOCK_SUCCESS, {"机器人上桩成功,正在充电", "Charging started successfully"}},
    {TTSStrings::TTS_KEY_DOCK_FAIL, {"机器人上桩失败", "Robot failed to dock."}},
    {TTSStrings::TTS_KEY_LEAVE_UDOCK, {"机器人正在下桩，请注意", "Attention please, Robot is leaving charging pile!"}},
    {TTSStrings::TTS_KEY_LEAVE_FAIL, {"机器人下桩失败", "Robot failed to leave charging pile!"}},
    {TTSStrings::TTS_KEY_LEAVE_SUCCESS, {"机器人下桩成功", "Robot leaved charging pile successfully!"}},
//    {TTSStrings::TTS_KEY_BEGAIN_AUTO_UDOCK, {"准备开始回充，请注意避让", "Prepare to start recharging, please pay attention to avoid"}},
//    {TTSStrings::TTS_KEY_IDENTIFY_SUCCESS, {"识别成功，开始运动", "Recognition successful, start moving"}},
//    {TTSStrings::TTS_KEY_BLUETOOTH_SUCCESS, {"蓝牙匹配成功", "Bluetooth match successful"}},
//    {TTSStrings::TTS_KEY_BEGAIN_DOCK, {"开始对桩", "Move on Pile"}},

//    {TTSStrings::TTS_KEY_IDENTIFY_FAIL, {"识别失败", "Recognize failed"}},
//    {TTSStrings::TTS_KEY_MOVE_FAIL, {"运动失败", "Move failed"}},
//    {TTSStrings::TTS_KEY_BLUETOOTH_FAIL, {"蓝牙匹配失败", "Bluetooth match failed"}},
//    {TTSStrings::TTS_KEY_CHARGE_FAIL, {"充电失败", "Charge Failed"}},
    {TTSStrings::TTS_KEY_RC_CHASSIS_LOCKED_BY_LIDAR, {"急刹车", "Emergency brake"}},
    {TTSStrings::TTS_KEY_RC_FRONT_SINGLE_LIDAR_FAILURE, {"前置单线雷达故障", "Front single line lidar failure"}},
    {TTSStrings::TTS_KEY_RC_MULTI_LIDAR_FAILURE, {"多线雷达故障", "Multi-line Lidar failure"}},
    {TTSStrings::TTS_KEY_RC_REAR_LIDAR_FAILURE, {"后置雷达故障", "Rear lidar failure"}},
    {TTSStrings::TTS_KEY_RC_AFTER_CHASSIS_LOCKED_BY_LIDAR, {"底盘软锁定，请后退解锁", "Chassis locked, please go back to unlock"}},
    {TTSStrings::TTS_KEY_STEEP_SLOPE, {"斜坡，请注意", "steep slope, please note"}},
    {TTSStrings::TTS_KEY_ENCOUNTER_OBSTACLE, {"机器人遇到障碍物，请移开","Encounter an obstacle，please remove it!"}},
    {TTSStrings::TTS_KEY_REMOTE_CONTROL_CHASSIS, {"遥控器控制底盘","remote control chassis!"}},
    {TTSStrings::TTS_KEY_SHTTPD_CONTROL_CHASSIS, {"网页控制底盘","shttpd control chassis!"}},
    {TTSStrings::TTS_KEY_PC_CONTROL_CHASSIS, {"远程控制底盘","pc control chassis!"}},
    {TTSStrings::TTS_KEY_RECHARGE_CONTROL_CHASSIS, {"回充控制底盘","recharge control chassis!"}},
    {TTSStrings::TTS_KEY_TEST_CONTROL_CHASSIS, {"测试控制底盘","test control chassis!"}},
    {TTSStrings::TTS_KEY_NAVIGATION_CONTROL_CHASSIS, {"导航控制底盘","gs control chassis!"}},
    {TTSStrings::TTS_KEY_TIMEOUT_CONTROL_CHASSIS, {"超时控制底盘","time out control chassis!"}},
    {TTSStrings::TTS_KEY_SPRAYER_SPRAYING, {"目前正在进行环境消杀，请注意避让","Environmental killing is in progress. Please avoid it"}},
    {TTSStrings::TTS_KEY_SERVER_ON_CONNECT, {"服务器连接成功","Server connected successfully!"}},
    {TTSStrings::TTS_KEY_SERVER_DISCONNECT, {"服务器断开连接","Server disconnected!"}}
};

std::string TTSStrings::text(int key) {
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    int tts_lng = shmrbt.appdata.tts_lng;
    if (tts_lng >= TTS_LANGUAGE_MAX) {
      log_error("%s tts_lng invalid: %d", __FUNCTION__, tts_lng);
      return "";
    }
    return (strings_map.count(key) ? strings_map[key][tts_lng] : "");
}

