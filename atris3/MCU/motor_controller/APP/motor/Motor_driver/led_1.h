#ifndef __LED_1_H__
#define __LED_1_H__

#include "stm32f4xx.h" 
#include "motor_driver.h"
#ifdef __cplusplus
extern "C" {
#endif

#define PE1_HIGHT		GPIO_SetBits((GPIO_STD_TypeDef*)GPIOE,GPIO_Pin_1)
#define PE1_LOW			GPIO_ResetBits((GPIO_STD_TypeDef*)GPIOE,GPIO_Pin_1)

#define LED_RED_ER2_OFF			GPIO_SetBits((GPIO_STD_TypeDef*)GPIOA,GPIO_Pin_6)
#define LED_RED_ER2_ON			GPIO_ResetBits((GPIO_STD_TypeDef*)GPIOA,GPIO_Pin_6)

void led_init(void);
void red_led_er_init(void);

#ifdef __cplusplus
}
#endif


#endif
