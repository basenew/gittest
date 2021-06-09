/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/

#include "hall_driver.h"
#include "motor_driver.h"
#include "electric_angle.h"
#include <rtthread.h>

volatile uint8_t motor1HallState, motor2HallState;
volatile uint8_t motor1HallTriger =5, motor2HallTriger =5;
/****************************************************************/
void motor_hall_interupt_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
	NVIC_InitStructure.NVIC_IRQChannel = MOTOR2_HALL_EXTI_IRQN;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}
/****************************************************************/
void motor1_hall_init(void)
{

	GPIO_STD_InitTypeDef               GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	RCC_AHB1PeriphClockCmd(	RCC_AHB1Periph_GPIOF,ENABLE); 
	/* Enable SYSCFG clock */

	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_HU_PIN | MOTOR1_HV_PIN | MOTOR1_HW_PIN;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_HU_PIN ;
	GPIO_Init(MOTOR1_HU_PORT, &GPIO_InitStructure);			

	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_HV_PIN ;
	GPIO_Init(MOTOR1_HV_PORT, &GPIO_InitStructure);			

	GPIO_InitStructure.GPIO_Pin 	= MOTOR1_HW_PIN ;
	GPIO_Init(MOTOR1_HW_PORT, &GPIO_InitStructure);			

	/* Connect EXTI Line0 to PA0 pin */
	SYSCFG_EXTILineConfig(MOTOR1_HU_PORT_PINSOURCE, MOTOR1_HU_PinSource);
	SYSCFG_EXTILineConfig(MOTOR1_HV_PORT_PINSOURCE, MOTOR1_HV_PinSource);
	SYSCFG_EXTILineConfig(MOTOR1_HW_PORT_PINSOURCE, MOTOR1_HW_PinSource);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = MOTOR1_HU_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = MOTOR1_HV_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = MOTOR1_HW_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}
/****************************************************************/
void motor2_hall_init(void)
{

	GPIO_STD_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	RCC_AHB1PeriphClockCmd(	RCC_AHB1Periph_GPIOF,ENABLE); 
	/* Enable SYSCFG clock */
	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_HU_PIN | MOTOR2_HV_PIN | MOTOR2_HW_PIN;


	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_HU_PIN ;
	GPIO_Init(MOTOR2_HU_PORT, &GPIO_InitStructure);			

	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_HV_PIN ;
	GPIO_Init(MOTOR2_HV_PORT, &GPIO_InitStructure);			

	GPIO_InitStructure.GPIO_Pin 	= MOTOR2_HW_PIN ;
	GPIO_Init(MOTOR2_HW_PORT, &GPIO_InitStructure);			

	/* Connect EXTI Line0 to PA0 pin */
	SYSCFG_EXTILineConfig(MOTOR2_HU_PORT_PINSOURCE, MOTOR2_HU_PinSource);
	SYSCFG_EXTILineConfig(MOTOR2_HV_PORT_PINSOURCE, MOTOR2_HV_PinSource);
	SYSCFG_EXTILineConfig(MOTOR2_HW_PORT_PINSOURCE, MOTOR2_HW_PinSource);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = MOTOR2_HU_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = MOTOR2_HV_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = MOTOR2_HW_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/****************************************************************/
void get_motor1_hall(void)
{
	uint8_t hall_u, hall_v, hall_w;
	hall_u = GPIO_ReadInputDataBit(MOTOR1_HU_PORT, MOTOR1_HU_PIN);
	hall_v = GPIO_ReadInputDataBit(MOTOR1_HV_PORT, MOTOR1_HV_PIN);
    hall_w = GPIO_ReadInputDataBit(MOTOR1_HW_PORT, MOTOR1_HW_PIN);
	motor1HallState = hall_u | (hall_v<<1) |(hall_w<<2);
}
/****************************************************************/
void get_motor2_hall(void)
{
	uint8_t hall_u, hall_v, hall_w;
	hall_u = GPIO_ReadInputDataBit(MOTOR2_HU_PORT, MOTOR2_HU_PIN);
	hall_v = GPIO_ReadInputDataBit(MOTOR2_HV_PORT, MOTOR2_HV_PIN);
	hall_w = GPIO_ReadInputDataBit(MOTOR2_HW_PORT, MOTOR2_HW_PIN);
	motor2HallState = hall_u | (hall_v<<1) |(hall_w<<2);
}
/****************************************************************/
void MOTOR_EXTI15_10_IRQHandler(void)
{
	rt_interrupt_enter();	
	if(EXTI_GetITStatus(MOTOR1_HU_EXTI_LINE) != RESET){
		EXTI_ClearITPendingBit(MOTOR1_HU_EXTI_LINE);
		motor1HallTriger = 0;
		get_motor1_hall();
		hall_electric_angle_4000();
	}
	if(EXTI_GetITStatus(MOTOR1_HV_EXTI_LINE) != RESET){
		EXTI_ClearITPendingBit(MOTOR1_HV_EXTI_LINE);
		motor1HallTriger = 1;
		get_motor1_hall();
		hall_electric_angle_4000();
	}
	if(EXTI_GetITStatus(MOTOR1_HW_EXTI_LINE) != RESET){
		EXTI_ClearITPendingBit(MOTOR1_HW_EXTI_LINE);
		motor1HallTriger = 2;
		get_motor1_hall();
		hall_electric_angle_4000();
	}
	
	if(EXTI_GetITStatus(MOTOR2_HU_EXTI_LINE) != RESET){
		EXTI_ClearITPendingBit(MOTOR2_HU_EXTI_LINE);
		motor2HallTriger = 0;
		get_motor2_hall();
		hall2_electric_angle_4000();
	}
	if(EXTI_GetITStatus(MOTOR2_HV_EXTI_LINE) != RESET){
		EXTI_ClearITPendingBit(MOTOR2_HV_EXTI_LINE);
		motor2HallTriger = 1;
		get_motor2_hall();
		hall2_electric_angle_4000();		
	}
	if(EXTI_GetITStatus(MOTOR2_HW_EXTI_LINE) != RESET){
		EXTI_ClearITPendingBit(MOTOR2_HW_EXTI_LINE);
		motor2HallTriger = 2;
		get_motor2_hall();
		hall2_electric_angle_4000();
	}
	rt_interrupt_leave();
}
/*===========================end of file====================*/
