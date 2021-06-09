/******************** (C) COPYRIGHT 2016 ********************
 * 文件名  ：.h
 * 描述    ： 应用函数库
 *
 * 实验平台：
 * 硬件连接：

***************************************************************/
#ifndef	_MOTOR_PROTECT_H_
#define	_MOTOR_PROTECT_H_

#include "stm32f4xx.h"


void motor_current_protect(void);
uint8_t get_over_current_status(void);
uint8_t get_over_current_status1(void);
uint8_t get_over_current_status2(void);
void reset_over_current_status(void);


uint8_t get_em_stop_state(void);
void em_stop_release_detect(void);
void receive_em_stop(void);
void set_over_current_value(uint16_t value);	
#endif

/******************* (C) COPYRIGHT 2016*****END OF FILE****/
