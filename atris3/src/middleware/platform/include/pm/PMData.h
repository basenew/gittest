#ifndef __PLATFORM_POWER__MANAGER_DATA_H__
#define __PLATFORM_POWER__MANAGER_DATA_H__

#include "config/config.h"

#define DF_REMOTE_OFF_LINE   -1
#define DF_REMOTE_ON    0
#define DF_REMOTE_ON_LINE    1
#define DF_REMOTE_CONTRUL   2
#define DF_REMOTE_ROCKER_NOT_ON_CENTER   3

#ifdef ENABLE_TEST_LOG
#define BATTERY_TEST
#endif

#define POWER_OFF_CMD   0xF0F0
#define POWER_OFF_MAGIC_CODE    666

#define DF_REMOTE_INFO_TIME 10.0F
#define DF_REMOTE_CYCLE 0.1F
#define DF_PWM_CHASSIS_LOCK_CYCLE 7.0F

#define DF_NORMALLY_OPEN_BUMPER 0 //防撞条常开式
#define DF_NORMALLY_CLOSED_BUMPER 1//防撞条常闭式

typedef enum RemoteControlMode_{
    SAFE_MODE = 0,
    STD_MODE,
    EMC_MODE

}RemoteControlMode;

typedef struct AtrisPowerCmd_
{
    int magic_code;//666
    int cmd;
}AtrisPowerCmd;

enum PmCtrlCmd
{
    PM_CMD_START_RESP = 0x81,
    PM_CMD_START = 0x82,
    PM_CMD_ECO_RESP = 0x83,
    PM_CMD_ECO = 0x84,
    PM_CMD_POWEROFF_RESP = 0x85,
    PM_CMD_POWEROFF = 0x86,
    PM_CMD_POWEROFF_FIN_RESP = 0x88,
    PM_CMD_POWEROFF_FIN = 0x87
};

enum PmMonitorCmd
{
    PM_CMD_MIDDLE_BRAKE = 0x38,
    PM_CMD_MIDDLE_FAN = 0x45,
    PM_INFO_MIDDLE_FAN = 0x46,
    PM_CMD_BOTTOM_FAN = 0x5B,
    PM_INFO_BOTTOM_FAN = 0x5C,
    PM_CMD_POWER_STATUS = 0x43,
    PM_CMD_POWER_IAP = 0x49,
    PM_INFO_POWER_IAP = 0x4A,
    PM_INFO_POWER_STATUS = 0x44,
    PM_CMD_SENSOR_LIQUID = 0x54,
    PM_CMD_SENSOR_HOST_TMP = 0x58,
    PM_CMD_SENSOR_HALL_CURRENT = 0x56,
    PM_CMD_SENSOR_HUMIDITY = 0x5A,
    PM_CMD_REMOTE_CONTROLLER_SPEED = 0X5E,
    PM_CMD_REMOTE_SBUS_356 = 0x60,
    PM_CMD_REMOTE_SBUS_78 = 0x62,
    PM_INFO_ELECTRODES = 0x64,
    PM_CMD_BLUETOOTH= 0x65,
    PM_INFO_BLUETOOTH= 0x66,
    PM_CMD_CHARGE_STATE= 0x67,
    PM_INFO_CHARGE_STATE= 0x68,
    PM_CMD_BUMPER_STATE= 0x72,
    PM_CMD_REMOTE_CONTROL_MODE= 0x73,
    PM_INFO_REMOTE_CONTROL_MODE= 0x74,
    PM_INFO_PROJECT_ID = 0x75,
    PM_CMD_PC_LEAVE_PILE= 0x69,
    PM_INFO_PC_LEAVE_PILE= 0x6A,
    PM_CMD_DOCK_LEAVE_PILE= 0x6B,
    PM_INFO_DOCK_LEAVE_PILE= 0x6C,
    PM_INFO_DOCK_FAIL_STATE= 0x6E,
    PM_INFO_SYSTEM_TIME = 0x77,
    PM_CMD_SYSTEM_TIME = 0x78,
    PM_CMD_ULTRASOUND_VER = 0x09,
    PM_INFO_ULTRASOUND_VER = 0x0A,
};
#define REMOTE_CTRL_PWM_CENTER          1500
#define REMOTE_CTRL_PWM_MAX_VALUE       500

#define REMOTE_CTRL_SBUS_CENTER         1000
#define REMOTE_CTRL_SBUS_MAX_VALUE      800
#define CALC_RIGHT_LINE_SPEED(pwm,center,max)   ((((float)pwm) - center) * Config::get_instance()->forward_max / max)   //最大1米/秒
#define CALC_LEFT_LINE_SPEED(pwm,center,max)   ((((float)pwm) - center) * Config::get_instance()->forward_max / max)   //最大1米/秒

#define CALC_WHEEL_FORWARD_SPEED_MAX(pwm,center,max)   ((((float)pwm) - center) * Config::get_instance()->forward_wheel_max / max)   //最大1米/秒
#define CALC_WHEEL_BACKWARD_SPEED_MAX(pwm,center,max)   ((((float)pwm) - center) * Config::get_instance()->backward_wheel_max / max)   //最大1米/秒
#define CALC_WHEEL_ANGULAR_MAX(pwm,center,max)           ((((float)pwm) - center) * Config::get_instance()->angular_wheel_max / max)   //最大1米/秒


#define REMOTE_CTRL_ABNOR_VALUL     100

#endif
