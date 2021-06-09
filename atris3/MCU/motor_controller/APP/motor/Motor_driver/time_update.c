/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/
#include "time_update.h"
#include "pid.h"
#include "motor_driver.h"

volatile uint32_t 	system_ticks =0;

/***************************************************/
uint32_t get_sys_ticks(void)
{
	return system_ticks;
}

/***************************************************/
//void pid_time_init(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

//  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);

//  TIM_TimeBaseStructure.TIM_Period = 1000 - 1;		//--1K HZ
//  TIM_TimeBaseStructure.TIM_Prescaler = 168/2 -1;
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);	
//  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
//  TIM_Cmd(TIM7, ENABLE);
//}
/***************************************************/
//void TIM7_IRQHandler(void)
//{
//	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
//	system_ticks++;
//}

/***************************************************/
void sys_ticks_time_plus(void)
{
	system_ticks++;
}



/******************* (C) COPYRIGHT 2016*****END OF FILE****/

