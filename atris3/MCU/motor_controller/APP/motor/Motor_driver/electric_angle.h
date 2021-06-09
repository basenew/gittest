/******************** (C) COPYRIGHT 2016 ********************
* Filename:
* Description:
* Platform:
* Other info:
* Author         Date          Notes
xiangbin.huang   2017.02.21    The first version template
***************************************************************/
#ifndef __ELE_ANGLE_H__
#define __ELE_ANGLE_H__


#include "stm32f4xx.h" 

#ifdef __cplusplus
extern "C" {
#endif
#define 	MECHANICAL_ANGLE	(3200/15)
#define 	MECHANICAL_ANGLE_F	((float)3200/15)
#define 	ANGLE_360	(360.0f)  
    
#define 	POLAR_LOGARITHM		(1)
#define 	ELE_OFFSET  		(MECHANICAL_ANGLE*90/360)
#define 	ELE_OFFSET2  		(MECHANICAL_ANGLE*90/360)
#define   	TEST_PWM			-500

int16_t hall_electric_angle_4000(void);
float encoder_electric_angle_4000(void);

int16_t hall2_electric_angle_4000(void);
float encoder2_electric_angle_4000(void);

uint8_t get_encoder_state(void);

#ifdef __cplusplus
}
#endif
#endif
