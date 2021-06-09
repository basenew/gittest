#ifndef _CHARGER_H_
#define _CHARGER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "board.h"
	

#define   PIN_POWER_KEY_IN_PE4                   GET_PIN(E, 4)			//电源开关按键检测
#define   POWER_KEY_IN_STATUS                 rt_pin_read(PIN_POWER_KEY_IN_PE4)
	

#define   PIN_POWER_12V_KEY_LED_PI9                  GET_PIN(I, 9)		//电源开关按键指示灯
	
#define  POWER_12V_KEY_LED_ENABLE                 rt_pin_write(PIN_POWER_12V_KEY_LED_PI9, PIN_HIGH)
#define  POWER_12V_KEY_LED_DISABLE                rt_pin_write(PIN_POWER_12V_KEY_LED_PI9, PIN_LOW)
#define  POWER_12V_KEY_LED_STATUS                 rt_pin_read(PIN_POWER_12V_KEY_LED_PI9)
	
	
	
	
#define   PIN_CHARGE_AUTO_CTL_PE15                   GET_PIN(E, 15)		//自动回充充电MOS控制
#define   PIN_CHARGE_MANUAL_CTL_PH9                  GET_PIN(H, 9)		//直充充电MOS控制
#define   PIN_CHARGE_POLE_CTL_PH11                   GET_PIN(H, 11)		//充电桩MOS控制

#define  CHARGE_AUTO_CTL_ENABLE                   rt_pin_write(PIN_CHARGE_AUTO_CTL_PE15, PIN_HIGH)
#define  CHARGE_AUTO_CTL_DISABLE                  rt_pin_write(PIN_CHARGE_AUTO_CTL_PE15, PIN_LOW)

#define  CHARGE_MANUAL_CTL_ENABLE                 rt_pin_write(PIN_CHARGE_MANUAL_CTL_PH9, PIN_HIGH)
#define  CHARGE_MANUAL_CTL_DISABLE                rt_pin_write(PIN_CHARGE_MANUAL_CTL_PH9, PIN_LOW)

#define  CHARGE_POLE_CTL_ENABLE                   rt_pin_write(PIN_CHARGE_POLE_CTL_PH11, PIN_HIGH)
#define  CHARGE_POLE_CTL_DISABLE                  rt_pin_write(PIN_CHARGE_POLE_CTL_PH11, PIN_LOW)
#define  CHARGE_POLE_CTL_STATUS                   rt_pin_read(PIN_CHARGE_POLE_CTL_PH11)

typedef struct
{
	uint8_t shutdown_report;
	uint8_t udock_standby_report;
	uint8_t normal_standby_wakeup_report;
	uint8_t manual_charge_report;
	uint8_t shutdown_start_by_key;
	uint8_t shutdown_start_by_host;
	uint8_t robot_mode;
	uint8_t light_mode;
	uint8_t power_mode;
	uint8_t motor_power_status;
}robot_t;

typedef struct
{
	uint8_t charger_source;
}charger_data_t;


typedef struct
{
	uint8_t charger_pole_report;
	uint8_t udock_pointup_finish;
	uint8_t pointdown_start;
	uint8_t udock_pointdown_finish;
}udock_t;

typedef enum
{
    CHARGER_MANUAL = 1,
    CHARGER_AUTO,
} charger_source_t;

typedef enum
{
    MODE_WORK = 0,
	MODE_STANDBY,
    MODE_SHUTDOWN,
	MODE_POWER_UP,
}robot_work_mode_t;

typedef enum
{
    LIGHT_WORK = 0,
	LIGHT_STANDBY,
}light_mode_t;

typedef enum
{
    MOTOR_WORK = 0,
	MOTOR_STANDBY = 1,
	MOTOR_DEFAULT = 0xff,
}motor_power_t;

int32_t power_management_init(void);		//电源管理初始化
uint8_t udock_get_status_report(void);		//获取回充当前信息上报状态标志
void power_down_clr_status_report(uint8_t status);	//清除对应电源上下电状态上报标志
uint8_t power_down_get_status_report(void);	//获取当前电源上下电状态上报标志
void udock_clr_pole_report(void);			//清除电极电测状态上报标志
void udock_clr_pointdown_finish(void);		//清除下桩完成状态上报标志
void udock_set_pointup_finish(void);		//设置上桩完成标志
void udock_set_pointdown_flag(void);		//设置下桩开始标志
void power_shutdown_set_by_host(void);		//执行主控关机请求
void power_standby_set(void);				//电源进入待机状态
void light_standby_set(void);				//灯效进入待机状态
void power_wakeup_set(void);				//电源唤醒
uint8_t light_mode_get(void);				//当前灯效状态获取
uint8_t power_mode_get(void);				//当前电源状态获取
uint8_t power_motor_status_get(void);		//驱动器电源状态获取
uint8_t robot_mode_get(void);				//机器当前工作状态获取

#ifdef __cplusplus
}
#endif


#endif
