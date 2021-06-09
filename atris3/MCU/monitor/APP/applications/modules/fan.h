#ifndef ____APP_MODULES_FAN_H__
#define ____APP_MODULES_FAN_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stdint.h"




#define   PIN_POWER_24V_FAN_PD14                     GET_PIN(D, 14)			//风扇电源

#define  POWER_24V_FAN_ENABLE                     rt_pin_write(PIN_POWER_24V_FAN_PD14, PIN_HIGH)
#define  POWER_24V_FAN_DISABLE                    rt_pin_write(PIN_POWER_24V_FAN_PD14, PIN_LOW)
#define  POWER_24V_FAN_STATUS                     rt_pin_read(PIN_POWER_24V_FAN_PD14)
	
#define   PIN_POWER_24V_FAN_PD14              GET_PIN(D, 14)			//风扇电源
#define   PIN_FAN_PWM_F_PD12                  GET_PIN(D, 12)			//??PWM_F
#define   PIN_FAN_PWM_R_PD13                  GET_PIN(D, 13)			//??PWM_F
#define   PIN_FAN_RD1_IN_PG2                  GET_PIN(G, 2)			    //???????
#define   PIN_FAN_RD2_IN_PG3                  GET_PIN(G, 3)			    //???????
#define   PIN_FAN_RD3_IN_PG4                  GET_PIN(G, 4)			    //???????
#define   PIN_FAN_RD4_IN_PG5                  GET_PIN(G, 5)			    //???????
	
	
#define  FAN_RD1_IN_STATUS                     rt_pin_read(PIN_FAN_RD1_IN_PG2)
#define  FAN_RD2_IN_STATUS                     rt_pin_read(PIN_FAN_RD2_IN_PG3)
#define  FAN_RD3_IN_STATUS                     rt_pin_read(PIN_FAN_RD3_IN_PG4)
#define  FAN_RD4_IN_STATUS                     rt_pin_read(PIN_FAN_RD4_IN_PG5)
	
enum
{
    FAN_SPEED_LVL0 = 0,
    FAN_SPEED_LVL1,
    FAN_SPEED_LVL2,
    FAN_SPEED_LVL3,
    FAN_SPEED_LVLMAX,
};

//who is controlling the fans
typedef enum
{
    FAN_CTRL_NOBODY = 0,
    FAN_CTRL_UNDERPAN,      
    FAN_CTRL_BATERRY,       
    FAN_CTRL_REMOTE,           
    FAN_CTRL_WHO_MAX,
} fan_who_t;


typedef struct
{
    fan_who_t   who;
    uint8_t     speed_level;
} fan_msg_t;

typedef struct
{
    uint8_t   speed_data;
    uint8_t   err_data;
} fan_status_t;



int32_t fan_init(void);                           //风扇初始化
void fan_remote_set(uint8_t speed);               //远程设置风扇速度
void fan_status_get(fan_status_t* fan_status);    //获取当前风扇状态

int32_t fan_mq_send(fan_msg_t * _msg);            //风扇状态上报
void fan_gpio_cfg_as_out(void);                   //风扇PWM配置为输出状态

	
#ifdef __cplusplus
}
#endif


#endif

