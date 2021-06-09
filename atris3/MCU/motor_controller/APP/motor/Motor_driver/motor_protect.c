/******************** (C) COPYRIGHT 2016 ********************
 * 文件名  ：.c
 * 描述    ： 应用函数库
 * 硬件资源：
 * 实验平台：
 * 硬件连接：
***************************************************************/
#include "motor_protect.h"
#include <rtthread.h>
#include <rthw.h>
#include "math.h"
#include "pid.h"
#include "pi.h"


extern float get_motor1_qs_current(void);
extern float get_motor2_qs_current(void);

#define MAX_CURRENT1	7.0f

#define FILTER1	(0.75f) //old 
#define FILTER2 (1.0f - FILTER1) //now

volatile uint8_t motor_over_current_status = 0;
volatile uint8_t motor1_over_current_status = 0;
volatile uint8_t motor2_over_current_status = 0;
/*************************************************************/

volatile 	uint8_t em_stop_status = 0;
volatile  	rt_tick_t em_tick = 0;

/*************************************************************/
//-------------------------------------------------
#define MAX_CURRENT		(MAX_CURRENT1 -2.0f)

volatile uint16_t 	over_current_value_ = MAX_CURRENT *1000.0f;
//-------------------------------------------------

void motor_current_protect(void)
{
	static float motor_current[2] ={0};
	float max_current = fabs(over_current_value_/1000.0f);
	
	motor_current[0] = motor_current[0]*FILTER1 + get_motor1_qs_current() *FILTER2;
	motor_current[1] = motor_current[1]*FILTER1 + get_motor2_qs_current() *FILTER2;
	if((fabs(motor_current[0]) > max_current) || (fabs(motor_current[1]) > max_current)){ 
		motor_over_current_status = 1; //过流保护
	}
	if(fabs(motor_current[0]) > max_current){
		motor1_over_current_status = 1;
	}
	if(fabs(motor_current[1]) > max_current){
		motor2_over_current_status = 1;
	}
}

void set_over_current_value(uint16_t value)
{
	over_current_value_ = value;
}

/*************************************************************/
uint8_t get_over_current_status(void)
{
	return motor_over_current_status;
}

/*************************************************************/
uint8_t get_over_current_status1(void)
{
	return motor1_over_current_status;
}

/*************************************************************/
uint8_t get_over_current_status2(void)
{
	return motor2_over_current_status;
}

/*************************************************************/
void reset_over_current_status(void)
{
	motor_over_current_status = 0;
	motor1_over_current_status = 0;
	motor2_over_current_status = 0;
}

//--------急停相关
/*************************************************************/
uint8_t get_em_stop_state(void)
{
	return em_stop_status;
}
/*************************************************************/
void em_stop_release_detect(void)
{
	if(rt_tick_get() - em_tick >2000){
		if(em_stop_status !=0){
			em_stop_status = 0;
			//
			pid_param_init_all();
			pi_init_motor1();
			pi_init_motor2();
			rt_thread_delay(100);
		}
	}
}
/*************************************************************/
void receive_em_stop(void)
{
	em_stop_status = 1;
	em_tick = rt_tick_get();
}

/******************* (C) COPYRIGHT 2016*****END OF FILE****/
