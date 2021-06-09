
/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/
#ifndef _HALL_DRIVER_H_
#define _HALL_DRIVER_H_
#include "stm32f4xx.h" 
#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR1_HU_PORT								((GPIO_STD_TypeDef*)GPIOF)
#define MOTOR1_HU_PIN								(GPIO_Pin_13)
#define MOTOR1_HU_PinSource						    GPIO_PinSource13
#define MOTOR1_HU_PORT_PINSOURCE					EXTI_PortSourceGPIOF
#define MOTOR1_HU_EXTI_LINE							EXTI_Line13
#define MOTOR1_HU_EXTI_IRQN  						EXTI15_10_IRQn;

#define MOTOR1_HV_PORT								((GPIO_STD_TypeDef*)GPIOF)
#define MOTOR1_HV_PIN								(GPIO_Pin_15)
#define MOTOR1_HV_PinSource						    GPIO_PinSource15
#define MOTOR1_HV_PORT_PINSOURCE					EXTI_PortSourceGPIOF
#define MOTOR1_HV_EXTI_LINE							EXTI_Line15
#define MOTOR1_HV_EXTI_IRQN  						EXTI15_10_IRQn;

#define MOTOR1_HW_PORT								((GPIO_STD_TypeDef*)GPIOF)
#define MOTOR1_HW_PIN								(GPIO_Pin_14)
#define MOTOR1_HW_PinSource						    GPIO_PinSource14
#define MOTOR1_HW_PORT_PINSOURCE					EXTI_PortSourceGPIOF
#define MOTOR1_HW_EXTI_LINE							EXTI_Line14
#define MOTOR1_HW_EXTI_IRQN  						EXTI15_10_IRQn;

#define MOTOR2_HU_PORT								((GPIO_STD_TypeDef*)GPIOF)
#define MOTOR2_HU_PIN								(GPIO_Pin_10)
#define MOTOR2_HU_PinSource						    GPIO_PinSource10
#define MOTOR2_HU_PORT_PINSOURCE					EXTI_PortSourceGPIOF
#define MOTOR2_HU_EXTI_LINE							EXTI_Line10

#define MOTOR2_HV_PORT								((GPIO_STD_TypeDef*)GPIOF)
#define MOTOR2_HV_PIN								(GPIO_Pin_12)
#define MOTOR2_HV_PinSource						    GPIO_PinSource12
#define MOTOR2_HV_PORT_PINSOURCE					EXTI_PortSourceGPIOF
#define MOTOR2_HV_EXTI_LINE							EXTI_Line12

#define MOTOR2_HW_PORT								((GPIO_STD_TypeDef*)GPIOF)
#define MOTOR2_HW_PIN								(GPIO_Pin_11)
#define MOTOR2_HW_PinSource						    GPIO_PinSource11
#define MOTOR2_HW_PORT_PINSOURCE					EXTI_PortSourceGPIOF
#define MOTOR2_HW_EXTI_LINE							EXTI_Line11

#define MOTOR2_HALL_EXTI_IRQN  						EXTI15_10_IRQn; 


void motor_hall_interupt_init(void);
void motor1_hall_init(void);
void motor2_hall_init(void);
void get_motor1_hall(void);
void get_motor2_hall(void);

#ifdef __cplusplus
}
#endif
#endif

/*===========================end of file====================*/
