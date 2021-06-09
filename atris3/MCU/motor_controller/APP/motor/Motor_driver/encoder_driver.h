/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/
#ifndef _ENCODER_DRIVER_H_
#define _ENCODER_DRIVER_H_


#include "stm32f4xx.h" 
#include "misc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_syscfg.h"
#ifdef __cplusplus
extern "C" {
#endif

#define DF_MOTOR1_ENCODE_TIM     					TIM4
#define DF_MOTOR1_ENCODE_RCC_APB_PERIPH_TIM     	RCC_APB1Periph_TIM4

#define DF_MOTOR1_ENCODE_RCC_APB_PERIPH_GPIO    	RCC_AHB1Periph_GPIOD
#define DF_MOTOR1_ENCODE_GPIO    					((GPIO_STD_TypeDef*)GPIOD)

#define DF_MOTOR1_ENCODE_A_PIN    					GPIO_Pin_12
#define DF_MOTOR1_ENCODE_B_PIN    					GPIO_Pin_13

#define DF_MOTOR1_ENCODE_A_PIN_SOURCE    			GPIO_PinSource12
#define DF_MOTOR1_ENCODE_B_PIN_SOURCE     			GPIO_PinSource13

#define DF_MOTOR1_ENCODE_GPIO_AF_TIM				GPIO_AF_TIM4

#define DF_MOTOR2_ENCODE_TIM     					TIM2
#define DF_MOTOR2_ENCODE_RCC_APB_PERIPH_TIM     	RCC_APB1Periph_TIM2

#define DF_MOTOR2_ENCODE_RCC_APB_PERIPH_GPIO    	(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB)
#define DF_MOTOR2_ENCODE_A_GPIO    					((GPIO_STD_TypeDef*)GPIOA)
#define DF_MOTOR2_ENCODE_B_GPIO    					((GPIO_STD_TypeDef*)GPIOB)

#define DF_MOTOR2_ENCODE_A_PIN    					GPIO_Pin_15
#define DF_MOTOR2_ENCODE_B_PIN    					GPIO_Pin_3

#define DF_MOTOR2_ENCODE_A_PIN_SOURCE    			GPIO_PinSource15
#define DF_MOTOR2_ENCODE_B_PIN_SOURCE     			GPIO_PinSource3

#define DF_MOTOR2_ENCODE_GPIO_AF_TIM				GPIO_AF_TIM2


void motor1_encoder_init(void);
void motor2_encoder_init(void);


#ifdef __cplusplus
}
#endif

#endif

/*===========================end of file====================*/
