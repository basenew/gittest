/******************** (C) COPYRIGHT 2016********************
 * �ļ���  ��.c
 * ����    ��         
 * ��汾  ��ST3.5.0
*********************************************************/
#ifndef _TIM_UPDATE_H_
#define _TIM_UPDATE_H_

#include "stm32f4xx.h" 

uint32_t get_sys_ticks(void);

void pid_time_init(void);
void sys_ticks_time_plus(void);


#endif

/*===========================end of file====================*/

