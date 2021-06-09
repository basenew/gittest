/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/

#include "encoder_driver.h"


/**********************************************************/
void motor1_encoder_init(void)
{
	GPIO_STD_InitTypeDef         GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef        TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_PinAFConfig(DF_MOTOR1_ENCODE_GPIO,DF_MOTOR1_ENCODE_A_PIN_SOURCE,DF_MOTOR1_ENCODE_GPIO_AF_TIM);
	GPIO_PinAFConfig(DF_MOTOR1_ENCODE_GPIO,DF_MOTOR1_ENCODE_B_PIN_SOURCE,DF_MOTOR1_ENCODE_GPIO_AF_TIM);

	GPIO_InitStructure.GPIO_Pin = DF_MOTOR1_ENCODE_A_PIN | DF_MOTOR1_ENCODE_B_PIN; 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
	GPIO_Init(DF_MOTOR1_ENCODE_GPIO,&GPIO_InitStructure); 


	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(DF_MOTOR1_ENCODE_TIM, &TIM_TimeBaseStructure); 

	TIM_EncoderInterfaceConfig(	DF_MOTOR1_ENCODE_TIM, 
								TIM_EncoderMode_TI12,
								TIM_ICPolarity_BothEdge,
								TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0xF;  
	TIM_ICInit(DF_MOTOR1_ENCODE_TIM, &TIM_ICInitStructure);
	TIM_ClearFlag(DF_MOTOR1_ENCODE_TIM, TIM_FLAG_Update);  
	TIM_ITConfig(DF_MOTOR1_ENCODE_TIM, TIM_IT_Update, ENABLE); 
	DF_MOTOR1_ENCODE_TIM->CNT = 0;
	TIM_Cmd(DF_MOTOR1_ENCODE_TIM, ENABLE);  
}

/**********************************************************/
void motor2_encoder_init(void)
{
	GPIO_STD_InitTypeDef         GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef        TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_PinAFConfig(DF_MOTOR2_ENCODE_A_GPIO,DF_MOTOR2_ENCODE_A_PIN_SOURCE,DF_MOTOR2_ENCODE_GPIO_AF_TIM);
	GPIO_PinAFConfig(DF_MOTOR2_ENCODE_B_GPIO,DF_MOTOR2_ENCODE_B_PIN_SOURCE,DF_MOTOR2_ENCODE_GPIO_AF_TIM);


	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
	GPIO_InitStructure.GPIO_Pin = DF_MOTOR2_ENCODE_A_PIN; 	
	GPIO_Init(DF_MOTOR2_ENCODE_A_GPIO,&GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = DF_MOTOR2_ENCODE_B_PIN; 	
	GPIO_Init(DF_MOTOR2_ENCODE_B_GPIO,&GPIO_InitStructure); 


	TIM_TimeBaseStructure.TIM_Period = 0xffff;// DF_MOTOR_ENCODE_PERIOD - 1; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;//TIM_CounterMode_Up;  
	TIM_TimeBaseInit(DF_MOTOR2_ENCODE_TIM, &TIM_TimeBaseStructure); 
	TIM_EncoderInterfaceConfig(	DF_MOTOR2_ENCODE_TIM, 
								TIM_EncoderMode_TI12,
								TIM_ICPolarity_BothEdge,
								TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0xF;  
	TIM_ICInit(DF_MOTOR2_ENCODE_TIM, &TIM_ICInitStructure);
	TIM_ClearFlag(DF_MOTOR2_ENCODE_TIM, TIM_FLAG_Update);  
	TIM_ITConfig(DF_MOTOR2_ENCODE_TIM, TIM_IT_Update, ENABLE); 
	DF_MOTOR2_ENCODE_TIM->CNT = 0;
	TIM_Cmd(DF_MOTOR2_ENCODE_TIM, ENABLE);  
}

/*===========================end of file====================*/
