/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/
#ifndef __ADC_SAMPLE_H__
#define __ADC_SAMPLE_H__

#include "stm32f4xx.h" 
#include "misc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_syscfg.h"
#define  DF_ADC_SIZE 7

#define DF_MOTOR1_IU_PERIPH_GPIO 		RCC_AHB1Periph_GPIOC
#define DF_MOTOR1_IU_GPIO 				((GPIO_STD_TypeDef*)GPIOC)
#define DF_MOTOR1_IU_PIN  				GPIO_Pin_0
#define DF_MOTOR1_IU_CHANNEL			ADC_Channel_10
#define DF_MOTOR1_IU_LOCAL				0

#define DF_MOTOR1_IV_PERIPH_GPIO 		RCC_AHB1Periph_GPIOC
#define DF_MOTOR1_IV_GPIO 				((GPIO_STD_TypeDef*)GPIOC)
#define DF_MOTOR1_IV_PIN  				GPIO_Pin_2
#define DF_MOTOR1_IV_CHANNEL			ADC_Channel_12
#define DF_MOTOR1_IV_LOCAL				1

#define DF_MOTOR1_IW_PERIPH_GPIO 		RCC_AHB1Periph_GPIOC
#define DF_MOTOR1_IW_GPIO 				((GPIO_STD_TypeDef*)GPIOC)
#define DF_MOTOR1_IW_PIN  				GPIO_Pin_3
#define DF_MOTOR1_IW_CHANNEL			ADC_Channel_13
#define DF_MOTOR1_IW_LOCAL				2



#define DF_MOTOR2_IU_PERIPH_GPIO 		RCC_AHB1Periph_GPIOF
#define DF_MOTOR2_IU_GPIO 				((GPIO_STD_TypeDef*)GPIOF)
#define DF_MOTOR2_IU_PIN  				GPIO_Pin_6
#define DF_MOTOR2_IU_CHANNEL			ADC_Channel_4
#define DF_MOTOR2_IU_LOCAL				3

#define DF_MOTOR2_IV_PERIPH_GPIO 		RCC_AHB1Periph_GPIOF
#define DF_MOTOR2_IV_GPIO 				((GPIO_STD_TypeDef*)GPIOF)
#define DF_MOTOR2_IV_PIN  				GPIO_Pin_7
#define DF_MOTOR2_IV_CHANNEL			ADC_Channel_5
#define DF_MOTOR2_IV_LOCAL				4

#define DF_MOTOR2_IW_PERIPH_GPIO 		RCC_AHB1Periph_GPIOF
#define DF_MOTOR2_IW_GPIO 				((GPIO_STD_TypeDef*)GPIOF)
#define DF_MOTOR2_IW_PIN  				GPIO_Pin_8
#define DF_MOTOR2_IW_CHANNEL			ADC_Channel_6
#define DF_MOTOR2_IW_LOCAL				5

#define DF_BUS_U_PERIPH_GPIO 		RCC_AHB1Periph_GPIOF
#define DF_BUS_U_GPIO 				((GPIO_STD_TypeDef*)GPIOF)
#define DF_BUS_U_PIN  				GPIO_Pin_5
#define DF_BUS_U_CHANNEL			ADC_Channel_15
#define DF_BUS_U_LOCAL				6

#define DF_ADC_DMA_PERIPH 		RCC_AHB1Periph_DMA2
#define DF_ADC_PERIPH     		RCC_APB2Periph_ADC3
#define DF_ADC     				ADC3
#define DF_ADC_DR    			ADC3->DR

#define DF_ADC_DMA_CHANNEL		DMA_Channel_2
#define DF_ADC_STREAM 			DMA2_Stream0


#define TEAMP0_GPIO_RCC 		RCC_AHB1Periph_GPIOF
#define TEAMP0_GPIO 			((GPIO_STD_TypeDef*)GPIOF)
#define TEAMP0_GPIO_PIN 		GPIO_Pin_3
#define TEAMP0_GPIO_CHANNEL		ADC_Channel_9
#define TEAMP0_GPIO_SEQ			7

#define TEAMP1_GPIO_RCC 		RCC_AHB1Periph_GPIOF
#define TEAMP1_GPIO 			((GPIO_STD_TypeDef*)GPIOF)
#define TEAMP1_GPIO_PIN 		GPIO_Pin_4
#define TEAMP1_GPIO_CHANNEL		ADC_Channel_14
#define TEAMP1_GPIO_SEQ			8


#ifdef __cplusplus
extern "C" {
#endif

#define CAL_HIGHT		GPIO_SetBits((GPIO_STD_TypeDef*)GPIOD,GPIO_Pin_0)
#define CAL_LOW			GPIO_ResetBits((GPIO_STD_TypeDef*)GPIOD,GPIO_Pin_0)	
	
void adc_current_init(void);	
void get_motor1_current(float *curent_a, float *curent_b, float *curent_c);
void get_motor2_current(float *curent_a, float *curent_b, float *curent_c);
void set_votage_offset(void);

	
#ifdef __cplusplus
}
#endif
#endif

/*===========================end of file====================*/
