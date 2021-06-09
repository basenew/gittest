#ifndef __APP_MODULES_SONAR_H__
#define __APP_MODULES_SONAR_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stdint.h"

#define  SONAR_RS485_DIRPIN_KS104_PI10              GET_PIN(I, 10)
	
#define  PIN_POWER_12V_KS104_1_PF11                 GET_PIN(F, 11)
#define  PIN_POWER_12V_KS104_2_PH2                  GET_PIN(H, 2)
#define  PIN_POWER_12V_KS104_3_PH3                  GET_PIN(H, 3)
#define  PIN_POWER_12V_KS104_4_PA0                  GET_PIN(A, 0)

#define  POWER_12V_KS104_1_ENABLE                     rt_pin_write(PIN_POWER_12V_KS104_1_PF11, PIN_HIGH)
#define  POWER_12V_KS104_1_DISABLE                    rt_pin_write(PIN_POWER_12V_KS104_1_PF11, PIN_LOW)
#define  POWER_12V_KS104_1_STATUS                     rt_pin_read(PIN_POWER_12V_KS104_1_PF11)

#define  POWER_12V_KS104_2_ENABLE                     rt_pin_write(PIN_POWER_12V_KS104_2_PH2, PIN_HIGH)
#define  POWER_12V_KS104_2_DISABLE                    rt_pin_write(PIN_POWER_12V_KS104_2_PH2, PIN_LOW)
#define  POWER_12V_KS104_2_STATUS                     rt_pin_read(PIN_POWER_12V_KS104_2_PH2)

#define  POWER_12V_KS104_3_ENABLE                     rt_pin_write(PIN_POWER_12V_KS104_3_PH3, PIN_HIGH)
#define  POWER_12V_KS104_3_DISABLE                    rt_pin_write(PIN_POWER_12V_KS104_3_PH3, PIN_LOW)
#define  POWER_12V_KS104_3_STATUS                     rt_pin_read(PIN_POWER_12V_KS104_3_PH3)

#define  POWER_12V_KS104_4_ENABLE                     rt_pin_write(PIN_POWER_12V_KS104_4_PA0, PIN_HIGH)
#define  POWER_12V_KS104_4_DISABLE                    rt_pin_write(PIN_POWER_12V_KS104_4_PA0, PIN_LOW)
#define  POWER_12V_KS104_4_STATUS                     rt_pin_read(PIN_POWER_12V_KS104_4_PA0)
	
	
	
int32_t sonar_init(void);                     //超声初始化
void sonar_get_data(uint16_t* sonar_data);    //获取超声当前数据

	
	
#ifdef __cplusplus
}
#endif


#endif




