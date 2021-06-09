#ifndef __APP_LIGHT_EFFECT_H__
#define __APP_LIGHT_EFFECT_H__

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif


//#define   PIN_POWER_12V_LIGHT_PB0                    GET_PIN(B, 0)			//照明灯
//#define   PIN_POWER_12V_CAM_LIGHT_PC9                GET_PIN(C, 9)			//云台补充灯 12V？24V

//#define  POWER_12V_LIGHT_ENABLE                   rt_pin_write(PIN_POWER_12V_LIGHT_PB0, PIN_HIGH)
//#define  POWER_12V_LIGHT_DISABLE                  rt_pin_write(PIN_POWER_12V_LIGHT_PB0, PIN_LOW)
//#define  POWER_12V_LIGHT_STATUS                   rt_pin_read(PIN_POWER_12V_LIGHT_PB0)

//#define  POWER_12V_CAM_LIGHT_ENABLE               rt_pin_write(PIN_POWER_12V_CAM_LIGHT_PC9, PIN_HIGH)
//#define  POWER_12V_CAM_LIGHT_DISABLE              rt_pin_write(PIN_POWER_12V_CAM_LIGHT_PC9, PIN_LOW)
//#define  POWER_12V_CAM_LIGHT_STATUS               rt_pin_read(PIN_POWER_12V_CAM_LIGHT_PC9)


typedef enum
{
    LIGHT_WORK = 0,
	LIGHT_STANDBY,
}light_mode_t;

typedef enum
{
    C_IDLE = 0x00, //杂色
    C_RED,         //红色
    C_YELLOW,      //黄色
    C_GREEN,       //绿色
    C_CYAN,        //青色
    C_BLUE,        //蓝色
    C_PURPLE,      //紫色
    C_WHITE,       //白色
    C_BLACK,       //黑色(熄灭)
} light_color_t;	

//触发灯效的事件。有优先级，优先级高的可以打断低的
typedef enum
{
    LIGHT_EVT_DISABLE = 0,         // 灯板关闭(不可用)状态
    LIGHT_EVT_BAT_CHARGE_ING = 1,  // 电池充电中，绿色呼吸
    LIGHT_EVT_BAT_CHARGE_FULL = 2, // 电池充满，绿色常亮
    LIGHT_EVT_WARN = 3,            // 系统警告，红灯常亮
    LIGHT_EVT_CONTROL = 4,         // 主控控制
    LIGHT_EVT_STANDBY = 5,         // 系统待机，蓝色呼吸
    LIGHT_EVT_WORK = 6,            // 系统工作正常，蓝灯常亮
    LIGHT_EVT_MAX,
} light_event_t;

typedef struct
{
    light_event_t evt;
    int32_t buffer[1];
} light_msg_t;


typedef struct
{
	uint8_t action;
	light_color_t color;
	uint8_t lum_max;
	uint8_t ctr_right;
} light_status_t;

int32_t light_post_event(light_event_t _evt, int32_t _status);   //灯效设置
int32_t light_effect_init(void);                                 //灯效控制初始化
int32_t light_remote_set(light_status_t light_status);           //远程设置灯效
light_status_t* light_remote_get(void);                          //远程灯效状态获取
uint32_t light_get_evt_bits(void);                               //获取灯效事件

#ifdef __cplusplus
}
#endif

#endif
