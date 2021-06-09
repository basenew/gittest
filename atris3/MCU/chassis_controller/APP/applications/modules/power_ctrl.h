#ifndef __APP_MODULES_POWER_CTRL_H__
#define __APP_MODULES_POWER_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "board.h"
	
#define   PIN_POWER_3V3_SYS_PF13                     GET_PIN(F, 13)			//232/485、温湿度传感器、电压检测、电流检测等芯片电源
#define   PIN_POWER_12V_RES1_PF14                    GET_PIN(F, 14)			//预留12V电源
#define   PIN_POWER_12V_RES2_PG0                     GET_PIN(G, 0)			//预留12V电源
	
#define   PIN_POWER_24V_DRIVER0_PA0                  GET_PIN(A, 0)			//驱动器电源
#define   PIN_POWER_24V_DRIVER1_PH2                  GET_PIN(H, 2)			//驱动器电源
#define   PIN_POWER_24V_DRIVER2_PH3                  GET_PIN(H, 3)			//驱动器电源
#define   PIN_POWER_24V_DRIVER3_PA6                  GET_PIN(A, 6)			//驱动器电源
#define   PIN_POWER_24V_DRIVER4_PF15                 GET_PIN(F, 15)			//驱动器电源
#define   PIN_POWER_24V_DRIVER5_PG1                  GET_PIN(G, 1)			//驱动器电源
#define   PIN_POWER_24V_DRIVER6_PE8                  GET_PIN(E, 8)			//驱动器电源
#define   PIN_POWER_24V_DRIVER7_PE10                 GET_PIN(E, 10)			//驱动器电源

#define   PIN_POWER_12V_TDK_PF1                      GET_PIN(F, 1)			//TDK12V电源
#define   PIN_POWER_24V_TDK_PI11                     GET_PIN(I, 11)			//TDK24V电源

#define   PIN_POWER_5V9_REMOTE_PC3                   GET_PIN(C, 3)			//遥控接收机电源



/////////////////
#define   POWER_3V3_SYS                     (1<<0)			//232/485、温湿度传感器、电压检测、电流检测等芯片电源

#define   POWER_24V_DRIVER0                  (1<<1)			//驱动器电源
#define   POWER_24V_DRIVER1                  (1<<2)			//驱动器电源
#define   POWER_24V_DRIVER2                  (1<<3)			//驱动器电源
#define   POWER_24V_DRIVER3                  (1<<4)			//驱动器电源
#define   POWER_24V_DRIVER4                 (1<<5)			//驱动器电源
#define   POWER_24V_DRIVER5                  (1<<6)			//驱动器电源
#define   POWER_24V_DRIVER6                  (1<<7)			//驱动器电源
#define   POWER_24V_DRIVER7                 (1<<8)			//驱动器电源

#define   POWER_24V_RIGHT_TEAR_STEER                   (1<<1)			//驱动器电源
#define   POWER_24V_RIGHT_TEAR_DIRECT                  (1<<2)			//驱动器电源
#define   POWER_24V_RIGHT_FRONT_STEER                  (1<<3)			//驱动器电源
#define   POWER_24V_RIGHT_FRONT_DIRECT                 (1<<4)			//驱动器电源
#define   POWER_24V_LEFT_FRONT_STEER                   (1<<5)			//驱动器电源
#define   POWER_24V_LEFT_FRONT_DIRECT                  (1<<6)			//驱动器电源
#define   POWER_24V_LEFT_TEAR_STEER                    (1<<7)			//驱动器电源
#define   POWER_24V_LEFT_TEAR_DIRECT                   (1<<8)			//驱动器电源


#define  POWER_24V_DRIVER0_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER0_PA0, PIN_HIGH)
#define  POWER_24V_DRIVER1_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER1_PH2, PIN_HIGH)
#define  POWER_24V_DRIVER2_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER2_PH3, PIN_HIGH)
#define  POWER_24V_DRIVER3_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER3_PA6, PIN_HIGH)
#define  POWER_24V_DRIVER4_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER4_PF15, PIN_HIGH)
#define  POWER_24V_DRIVER5_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER5_PG1, PIN_HIGH)
#define  POWER_24V_DRIVER6_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER6_PE8, PIN_HIGH)
#define  POWER_24V_DRIVER7_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER7_PE10, PIN_HIGH)


#define   POWER_24V_DRIVER_ALL               (POWER_24V_DRIVER0 | POWER_24V_DRIVER1 | POWER_24V_DRIVER2 | POWER_24V_DRIVER3 | \
                                             POWER_24V_DRIVER4 | POWER_24V_DRIVER5 | POWER_24V_DRIVER6 | POWER_24V_DRIVER7)  
#define   POWER_24V_MOTOR_DRIVER_ALL               (POWER_24V_DRIVER0 | POWER_24V_DRIVER1 | POWER_24V_DRIVER2 | POWER_24V_DRIVER3 | \
                                             POWER_24V_DRIVER4 | POWER_24V_DRIVER5 | POWER_24V_DRIVER6 | POWER_24V_DRIVER7)  

#define   POWER_12V_RES1                   (1<<9)			//预留12V电源
#define   POWER_12V_RES2                     (1<<10)			//预留12V电源
	


#define   POWER_12V_TDK                      (1<<11)			//TDK12V电源
#define   POWER_24V_TDK                     (1<<12)			//TDK24V电源

#define   POWER_5V9_REMOTE                  (1<<13)			//遥控接收机电源


//-----------------------------------------------------------------------------------------------------------------------------
#define  POWER_3V3_SYS_ENABLE                     rt_pin_write(PIN_POWER_3V3_SYS_PF13, PIN_HIGH)
#define  POWER_3V3_SYS_DISABLE                    rt_pin_write(PIN_POWER_3V3_SYS_PF13, PIN_LOW)
#define  POWER_3V3_SYS_STATUS                     rt_pin_read(PIN_POWER_3V3_SYS_PF13)

#define  POWER_12V_RES1_ENABLE                    rt_pin_write(PIN_POWER_12V_RES1_PF14, PIN_HIGH)
#define  POWER_12V_RES1_DISABLE                   rt_pin_write(PIN_POWER_12V_RES1_PF14, PIN_LOW)
#define  POWER_12V_RES1_STATUS                    rt_pin_read(PIN_POWER_12V_RES1_PF14)

#define  POWER_12V_RES2_ENABLE                    rt_pin_write(PIN_POWER_12V_RES2_PG0, PIN_HIGH)
#define  POWER_12V_RES2_DISABLE                   rt_pin_write(PIN_POWER_12V_RES2_PG0, PIN_LOW)
#define  POWER_12V_RES2_STATUS                    rt_pin_read(PIN_POWER_12V_RES2_PG0)

#define  POWER_24V_DRIVER0_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER0_PA0, PIN_HIGH)
#define  POWER_24V_DRIVER0_DISABLE                rt_pin_write(PIN_POWER_24V_DRIVER0_PA0, PIN_LOW)
#define  POWER_24V_DRIVER0_STATUS                 rt_pin_read(PIN_POWER_24V_DRIVER0_PA0)

#define  POWER_24V_DRIVER1_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER1_PH2, PIN_HIGH)
#define  POWER_24V_DRIVER1_DISABLE                rt_pin_write(PIN_POWER_24V_DRIVER1_PH2, PIN_LOW)
#define  POWER_24V_DRIVER1_STATUS                 rt_pin_read(PIN_POWER_24V_DRIVER1_PH2)

#define  POWER_24V_DRIVER2_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER2_PH3, PIN_HIGH)
#define  POWER_24V_DRIVER2_DISABLE                rt_pin_write(PIN_POWER_24V_DRIVER2_PH3, PIN_LOW)
#define  POWER_24V_DRIVER2_STATUS                 rt_pin_read(PIN_POWER_24V_DRIVER2_PH3)

#define  POWER_24V_DRIVER3_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER3_PA6, PIN_HIGH)
#define  POWER_24V_DRIVER3_DISABLE                rt_pin_write(PIN_POWER_24V_DRIVER3_PA6, PIN_LOW)
#define  POWER_24V_DRIVER3_STATUS                 rt_pin_read(PIN_POWER_24V_DRIVER3_PA6)

#define  POWER_24V_DRIVER4_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER4_PF15, PIN_HIGH)
#define  POWER_24V_DRIVER4_DISABLE                rt_pin_write(PIN_POWER_24V_DRIVER4_PF15, PIN_LOW)
#define  POWER_24V_DRIVER4_STATUS                 rt_pin_read(PIN_POWER_24V_DRIVER4_PF15)

#define  POWER_24V_DRIVER5_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER5_PG1, PIN_HIGH)
#define  POWER_24V_DRIVER5_DISABLE                rt_pin_write(PIN_POWER_24V_DRIVER5_PG1, PIN_LOW)
#define  POWER_24V_DRIVER5_STATUS                 rt_pin_read(PIN_POWER_24V_DRIVER5_PG1)

#define  POWER_24V_DRIVER6_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER6_PE8, PIN_HIGH)
#define  POWER_24V_DRIVER6_DISABLE                rt_pin_write(PIN_POWER_24V_DRIVER6_PE8, PIN_LOW)
#define  POWER_24V_DRIVER6_STATUS                 rt_pin_read(PIN_POWER_24V_DRIVER6_PE8)

#define  POWER_24V_DRIVER7_ENABLE                 rt_pin_write(PIN_POWER_24V_DRIVER7_PE10, PIN_HIGH)
#define  POWER_24V_DRIVER7_DISABLE                rt_pin_write(PIN_POWER_24V_DRIVER7_PE10, PIN_LOW)
#define  POWER_24V_DRIVER7_STATUS                 rt_pin_read(PIN_POWER_24V_DRIVER7_PE10)

#define  POWER_12V_TDK_ENABLE                     rt_pin_write(PIN_POWER_12V_TDK_PF1, PIN_HIGH)
#define  POWER_12V_TDK_DISABLE                    rt_pin_write(PIN_POWER_12V_TDK_PF1, PIN_LOW)
#define  POWER_12V_TDK_STATUS                     rt_pin_read(PIN_POWER_12V_TDK_PF1)

#define  POWER_24V_TDK_ENABLE                     rt_pin_write(PIN_POWER_24V_TDK_PI11, PIN_HIGH)
#define  POWER_24V_TDK_DISABLE                    rt_pin_write(PIN_POWER_24V_TDK_PI11, PIN_LOW)
#define  POWER_24V_TDK_STATUS                     rt_pin_read(PIN_POWER_24V_TDK_PI11)

#define  POWER_5V9_REMOTE_ENABLE                  rt_pin_write(PIN_POWER_5V9_REMOTE_PC3, PIN_HIGH)
#define  POWER_5V9_REMOTE_DISABLE                 rt_pin_write(PIN_POWER_5V9_REMOTE_PC3, PIN_LOW)
#define  POWER_5V9_REMOTE_STATUS                  rt_pin_read(PIN_POWER_5V9_REMOTE_PC3)



//-----------------------------------------------------------------------------------------------------------------------------


enum
{
    POWER_DRIVER_LEFT_FRONT_STEER   = 0x00,
	POWER_DRIVER_LEFT_FRONT_TRAVEL,
	POWER_DRIVER_RIGHT_FRONT_STEER,
	POWER_DRIVER_RIGHT_FRONT_TRAVEL,
    POWER_DRIVER_LEFT_BACK_STEER,
	POWER_DRIVER_LEFT_BACK_TRAVEL,
	POWER_DRIVER_RIGHT_BACK_STEER,
	POWER_DRIVER_RIGHT_BACK_TRAVEL,
};

enum
{
	POWER_OFF = 0,
	POWER_ON = 1,
};


typedef enum
{
    POWRE_WORK = 0,
	POWER_STANDBY,
}power_mode_t;


int32_t power_ctrl_init(void);				//电源控制GPIO初始化
void power_ctrl_all_turn_on(void);			//打开所有受控电源
void power_ctrl_all_turn_off(void);			//关闭所有受控电源
void power_into_wrok(void);					//电源进入正常工作状态
void power_into_standby(void);				//电源进入待机状态
void power_up(void);						//电源执行待机唤醒动作
void power_up_driver(void);					//驱动器电源执行待机唤醒动作
void power_driver_ctrl(uint8_t driver_num, uint8_t power_state);	//驱动器电源控制接口
void power_set_status(uint32_t status);		//所有受控电源控制接口
void power_set(uint32_t status);			//设置所有受控电源状态
uint32_t power_get_status(void);			//获取当前所有电源状态

void power_set_single_power(int sw,int onoff); 	//单路电源开关状态设置
void motor_power_off(void);					//关闭驱动器电源
void motor_power_on(void);  				//打开驱动器电源

#ifdef __cplusplus
}
#endif


#endif



