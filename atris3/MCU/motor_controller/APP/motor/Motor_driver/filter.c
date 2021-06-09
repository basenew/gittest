/******************** (C) COPYRIGHT 2016 ********************
 * 文件名  ：filter.c
 * 描述    ：filter 应用函数库
 *          
 * 实验平台：
 * 硬件连接：

***************************************************************/

#include "filter.h"
#include <string.h>
#include "pid.h"
#include "pi.h"
#include "filter.h"
#include "time_update.h"

struct Window_filter_Stru  Window_filter_speed[2];

/**********************************************************/
void Window_filter_Init(void)
{
	memset(&Window_filter_speed[0], 0, sizeof(struct  Window_filter_Stru));
	Window_filter_speed[0].index = 0;
	memset(&Window_filter_speed[1], 0, sizeof(struct  Window_filter_Stru));
	Window_filter_speed[1].index = 0;	
}
/**********************************************************/
float Window_filter_Update(struct  Window_filter_Stru *filter, int32_t encoder)
{
	filter->speed = encoder - filter->buff[filter->index];
	if(filter->speed > 65535/2){
		filter->speed  -= 65535;
	}
	else if(filter->speed < -65535/2){
		filter->speed  += 65535;
	}
	filter->buff[filter->index] = encoder;
	filter->index++;
	filter->index %= WINDOW_LENGTH;
	return filter->speed;
}
/**********************************************************/
void Window_filter_Update_Motor1(void)
{
  static int32_t encoder;
  static uint8_t fisrt_in =1;
  uint16_t i =0;

  if(fisrt_in == 0){
	encoder = TIM4->CNT;
	Window_filter_Update(&Window_filter_speed[0], encoder);
    return;
  }
  else{
	fisrt_in =0;
    encoder = TIM4->CNT;
	for(i=0; i<WINDOW_LENGTH; i++){
		Window_filter_speed[0].buff[i] = encoder;
	}
  }
}
/**********************************************************/
void Window_filter_Update_Motor2(void)
{
  static int32_t encoder;
  static uint8_t fisrt_in =1;
  uint16_t i =0;

  if(fisrt_in == 0){
	encoder = TIM2->CNT;
	Window_filter_Update(&Window_filter_speed[1], encoder);
    return;
  }
  else{
	fisrt_in =0;
    encoder = TIM2->CNT;
	for(i=0; i<WINDOW_LENGTH; i++){
		Window_filter_speed[1].buff[i] = encoder;
	}
  }
}
/**********************************************************/

void Window_filter_Update_all(void) //run once per 1ms
{
	float speed_rpm[2];
	Window_filter_Update_Motor1();
	Window_filter_Update_Motor2();
	/*-- 单位转换--转每min--*/
	/*-- 1r = 1024*4 conts = 4096 counts 减速比*线数*倍频*/ 
	/*-- N counts/4096 -- 10ms = 1min/60/100 */
	/*-- N counts*6000/3200 = N * 1.875f /10 time（ms） --*/
	speed_rpm[0] =  Window_filter_speed[0].speed * 0.1875f;//速度转换成：电机输出端，每分钟多少转
	pi_set_damp1(-speed_rpm[0]);
	speed_rpm[1] =  Window_filter_speed[1].speed * 0.1875f;
	pi_set_damp2(-speed_rpm[1]);
}

/**********************************************************/

void Window_filter_Get_Speed(int16_t *speed1, int16_t *speed2)
{
	*speed1 = (int16_t)Window_filter_speed[0].speed;
	*speed2 = (int16_t)Window_filter_speed[1].speed;
}
/*===========================end of file====================*/
