/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/
#ifndef _MOTOR_DRIVEER_H_
#define _MOTOR_DRIVEER_H_

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



#define MOTOR1_BRK_IRQN	      						TIM1_BRK_TIM9_IRQn
#define MOTOR2_BRK_IRQN	      						TIM8_BRK_TIM12_IRQn

#define MOTOR1_TIM_UP_IRQN	      						TIM1_UP_TIM10_IRQn
#define MOTOR2_TIM_UP_IRQN	      						TIM8_UP_TIM13_IRQn

#define MOTOR1_RCC_APB_PERIPH							RCC_APB2Periph_TIM1
#define MOTOR2_RCC_APB_PERIPH							RCC_APB2Periph_TIM8

#define MOTOR1_GPIO_AF_TIM								GPIO_AF_TIM1
#define MOTOR2_GPIO_AF_TIM								GPIO_AF_TIM8

#define MOTOR1_TIM												TIM1
#define MOTOR2_TIM												TIM8

#define MOTOR1_ENABLE_PORT								((GPIO_STD_TypeDef*)GPIOD)
#define MOTOR1_ENABLE_PIN									(GPIO_Pin_3)

#define MOTOR1_PWM_UP_PORT								((GPIO_STD_TypeDef*)GPIOE)
#define MOTOR1_PWM_UP_PIN									(GPIO_Pin_9)
#define MOTOR1_PWM_UP_PinSource						GPIO_PinSource9
#define MOTOR1_PWM_UN_PORT								((GPIO_STD_TypeDef*)GPIOE)
#define MOTOR1_PWM_UN_PIN									(GPIO_Pin_8)
#define MOTOR1_PWM_UN_PinSource						GPIO_PinSource8

#define MOTOR1_PWM_VP_PORT								((GPIO_STD_TypeDef*)GPIOE)
#define MOTOR1_PWM_VP_PIN									(GPIO_Pin_11)
#define MOTOR1_PWM_VP_PinSource						GPIO_PinSource11
#define MOTOR1_PWM_VN_PORT								((GPIO_STD_TypeDef*)GPIOE)
#define MOTOR1_PWM_VN_PIN									(GPIO_Pin_10)
#define MOTOR1_PWM_VN_PinSource						 GPIO_PinSource10

#define MOTOR1_PWM_WP_PORT								 ((GPIO_STD_TypeDef*)GPIOE)
#define MOTOR1_PWM_WP_PIN									 (GPIO_Pin_13)
#define MOTOR1_PWM_WP_PinSource						 GPIO_PinSource13
#define MOTOR1_PWM_WN_PORT								 ((GPIO_STD_TypeDef*)GPIOE)
#define MOTOR1_PWM_WN_PIN									 (GPIO_Pin_12)
#define MOTOR1_PWM_WN_PinSource						 GPIO_PinSource12


#define MOTOR1_ERROR_RCC_AHB_PERIPH					RCC_AHB1Periph_GPIOE
#define MOTOR1_ERROR_PORT									  ((GPIO_STD_TypeDef*)GPIOE)
#define MOTOR1_ERROR_PIN									  (GPIO_Pin_15)
#define MOTOR1_ERROR_PinSource						  GPIO_PinSource15

#define MOTOR2_ENABLE_PORT								((GPIO_STD_TypeDef*)GPIOC)
#define MOTOR2_ENABLE_PIN									(GPIO_Pin_11)

#define MOTOR2_PWM_UP_PORT								((GPIO_STD_TypeDef*)GPIOC)
#define MOTOR2_PWM_UP_PIN									(GPIO_Pin_6)
#define MOTOR2_PWM_UP_PinSource						GPIO_PinSource6
#define MOTOR2_PWM_UN_PORT								((GPIO_STD_TypeDef*)GPIOA)
#define MOTOR2_PWM_UN_PIN									(GPIO_Pin_5)
#define MOTOR2_PWM_UN_PinSource						 GPIO_PinSource5

#define MOTOR2_PWM_VP_PORT								((GPIO_STD_TypeDef*)GPIOC)
#define MOTOR2_PWM_VP_PIN									(GPIO_Pin_7)
#define MOTOR2_PWM_VP_PinSource						GPIO_PinSource7
#define MOTOR2_PWM_VN_PORT								((GPIO_STD_TypeDef*)GPIOB)
#define MOTOR2_PWM_VN_PIN									(GPIO_Pin_0)
#define MOTOR2_PWM_VN_PinSource						 GPIO_PinSource0

#define MOTOR2_PWM_WP_PORT								 ((GPIO_STD_TypeDef*)GPIOC)
#define MOTOR2_PWM_WP_PIN									 (GPIO_Pin_8)
#define MOTOR2_PWM_WP_PinSource						 GPIO_PinSource8
#define MOTOR2_PWM_WN_PORT								 ((GPIO_STD_TypeDef*)GPIOB)
#define MOTOR2_PWM_WN_PIN									 (GPIO_Pin_1)
#define MOTOR2_PWM_WN_PinSource						 GPIO_PinSource1

#define MOTOR2_ERROR_RCC_AHB_PERIPH					 RCC_AHB1Periph_GPIOA
#define MOTOR2_ERROR_PORT									   ((GPIO_STD_TypeDef*)GPIOE)
#define MOTOR2_ERROR_PIN									   (GPIO_Pin_6)
#define MOTOR2_ERROR_PinSource						   GPIO_PinSource6


#define MOTOR1_PWM_UL_ON		GPIO_SetBits(MOTOR1_PWM_UN_PORT, MOTOR1_PWM_UN_PIN)
#define MOTOR1_PWM_UL_OFF		GPIO_ResetBits(MOTOR1_PWM_UN_PORT, MOTOR1_PWM_UN_PIN)

#define MOTOR1_PWM_VL_ON		GPIO_SetBits(MOTOR1_PWM_VN_PORT, MOTOR1_PWM_VN_PIN)
#define MOTOR1_PWM_VL_OFF		GPIO_ResetBits(MOTOR1_PWM_VN_PORT, MOTOR1_PWM_VN_PIN)

#define MOTOR1_PWM_WL_ON		GPIO_SetBits(MOTOR1_PWM_WN_PORT, MOTOR1_PWM_WN_PIN)
#define MOTOR1_PWM_WL_OFF		GPIO_ResetBits(MOTOR1_PWM_WN_PORT, MOTOR1_PWM_WN_PIN)


#define MOTOR2_PWM_UL_ON		GPIO_SetBits(MOTOR2_PWM_UN_PORT, MOTOR2_PWM_UN_PIN)
#define MOTOR2_PWM_UL_OFF		GPIO_ResetBits(MOTOR2_PWM_UN_PORT, MOTOR2_PWM_UN_PIN)

#define MOTOR2_PWM_VL_ON		GPIO_SetBits(MOTOR2_PWM_VN_PORT, MOTOR2_PWM_VN_PIN)
#define MOTOR2_PWM_VL_OFF		GPIO_ResetBits(MOTOR2_PWM_VN_PORT, MOTOR2_PWM_VN_PIN)

#define MOTOR2_PWM_WL_ON		GPIO_SetBits(MOTOR2_PWM_WN_PORT, MOTOR2_PWM_WN_PIN)
#define MOTOR2_PWM_WL_OFF		GPIO_ResetBits(MOTOR2_PWM_WN_PORT, MOTOR2_PWM_WN_PIN)

#define PWM_FRQ					 (20000)
#define TS  					(unsigned short)(168*1000*1000/PWM_FRQ/2)

void motor1_gpio_init(void);
void motor1_tim_pwm_init(void);
void motor1_set_pwm(int pwm_u,int pwm_v, int pwm_w);

void motor2_gpio_init(void);
void motor2_tim_pwm_init(void);
void motor2_set_pwm(int pwm_u,int pwm_v, int pwm_w);


void motor1_control(int pwm);
void motor2_control(int pwm);
void motorOn(void);
void motorOff(void);
void foc_time_init(void);
#ifdef __cplusplus
}
#endif

#endif

/*===========================end of file====================*/
