/******************** (C) COPYRIGHT 2016 ********************
 * 文件名  ：filter.c
 * 描述    ：filter 应用函数库
 *          
 * 实验平台：
 * 硬件连接：

***************************************************************/
#ifndef _FILTER_H_
#define	_FILTER_H_

#include "stm32f4xx.h" 

#ifdef __cplusplus
extern "C" {
#endif

#define  WINDOW_LENGTH	100

struct Window_filter_Stru
{
	float speed;
	float variance;
	uint8_t  avaliable_length;
	uint16_t buff[WINDOW_LENGTH];
	uint16_t index;
};


void Window_filter_Init(void);
void Window_filter_Update_Motor1(void);
void Window_filter_Update_Motor2(void);
void Window_filter_Update_all(void);
void Window_filter_Get_Speed(int16_t *speed1, int16_t *speed2);


#ifdef __cplusplus
}
#endif

#endif 
/*===========================end of file====================*/
