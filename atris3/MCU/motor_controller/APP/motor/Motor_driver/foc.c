/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：main.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/
#include <rtthread.h>

#include "stm32f4xx.h" 
#include "svgen.h"
#include "ipark.h"
#include "clarke.h"
#include "park.h"
#include "motor_driver.h"
#include "led_1.h"
#include "electric_angle.h"
#include "pid.h"
#include "pi.h"
#include "adc_sample.h"
#include "filter.h"
//#include "odom.h"
#include "time_update.h"
#include "motor_protect.h"

#define HALF_PWM	2100
/*------------------变量声明-----------------------------------------------------------------------------------*/
SVGEN			svgen1={0};
IPARK			ipark1={0};
CLARKE			clarke1={0};
PARK			park1={0};

SVGEN			svgen2={0};
IPARK			ipark2={0};
CLARKE			clarke2={0};
PARK			park2={0};

extern void arm_sin_cos_f32(float theta, float * pSinVal, float * pCosVal);

/**********************************************************/
float get_motor1_qs_current(void)
{
	float ia, ib, ic;
	float sin_theta, cos_theta;
	//clarke
	arm_sin_cos_f32(ipark1.Angle, &sin_theta, &cos_theta);
	get_motor1_current(&ia, &ib, &ic);
	clarke1.As = ia;
	clarke1.Bs = ib;
	CLARKE_MACRO(clarke1);
	
	//park
	park1.Alpha = clarke1.Alpha;
	park1.Beta = clarke1.Beta;
	park1.Sine = sin_theta;
	park1.Cosine = cos_theta;
	PARK_MACRO(park1);
	return park1.Qs;
}
/**********************************************************/
float get_motor2_qs_current(void)
{
	float ia, ib, ic;
	float sin_theta, cos_theta;
	//clarke
	arm_sin_cos_f32(ipark2.Angle, &sin_theta, &cos_theta);
	//clarke
	get_motor2_current(&ia, &ib, &ic);
	clarke2.As = ib;
	clarke2.Bs = ia;
	CLARKE_MACRO(clarke2);
	
	//park
	park2.Alpha = clarke2.Alpha;
	park2.Beta = clarke2.Beta;
	park2.Sine = sin_theta;
	park2.Cosine = cos_theta;
	PARK_MACRO(park2);
	return park2.Qs;
}	
/**********************************************************/
void out_range_min_max(float *motor, int16_t range)
{
  if (*motor <-range){
    *motor = -range;
	}
  if(*motor >range){
    *motor = range;
	}
}
/**********************************************************/
void out_range_zero_offset(float *motor, int16_t range)
{
  if (*motor <0){
    *motor = *motor -range;
	}
  if(*motor >0){
    *motor = *motor +range;
	}
}
/**********************************************************/

void foc_level_four(void)
{
	//float ele_angle;
//	float ia, ib, ic;
    float sin_theta, cos_theta;
	static uint16_t count_position_control = 0;
	static float pid_out_qs = 0;
	static float pid_out_ds = 0;
	static float pi_out = 0;

	//angle caculate	
	//ele_angle = encoder_electric_angle_4000();//-106.6667f;
	ipark1.Angle = encoder_electric_angle_4000();//-106.6667f;
	arm_sin_cos_f32(ipark1.Angle, &sin_theta, &cos_theta);
	
	//position pi control
	count_position_control++;
	count_position_control %=10; //1KHZ
	if(count_position_control ==0){
		pi_set_measured1(TIM4->CNT);
		pi_out = pi_update1();
	}
	
	pid_out_qs = pi_out;
	pid_out_ds = 0;
	
	//ipark
	ipark1.Qs = pid_out_qs;
	ipark1.Ds = pid_out_ds;
	ipark1.Sine = sin_theta;
	ipark1.Cosine = cos_theta;
	IPARK_MACRO(ipark1);
	
	//svpwm
	svgen1.Ualpha = ipark1.Alpha;
	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1);
	
	out_range_min_max(&svgen1.Ta, 1600);
	out_range_min_max(&svgen1.Tb, 1600);
	out_range_min_max(&svgen1.Tc, 1600);
	if((get_over_current_status() !=0)  || (get_encoder_state() !=0)){
		motor1_set_pwm(0, 0, 0);
	}
	else{
		motor1_set_pwm(svgen1.Ta +HALF_PWM, svgen1.Tb +HALF_PWM, svgen1.Tc +HALF_PWM);
	}
	//pwm out
}
/**********************************************************/
void foc_level_four2(void)
{
	//float ele_angle;
//	float ia, ib, ic;
    float sin_theta, cos_theta;
	static uint16_t count_position_control = 0;
	static float pid_out_qs = 0;
	static float pid_out_ds = 0;
	static float pi_out 	= 0;
	
	//angle caculate
	//ele_angle = //-106.6667f;
	ipark2.Angle = encoder2_electric_angle_4000();
	arm_sin_cos_f32(ipark2.Angle, &sin_theta, &cos_theta);
	
	//position pi control
	count_position_control++;
	count_position_control %=10; //1KHZ
	if(count_position_control ==0){
		pi_set_measured2(TIM2->CNT);
		pi_out = pi_update2();
	}
	
	pid_out_qs = pi_out;
	pid_out_ds = 0;
	//ipark
	ipark2.Qs = pid_out_qs;
	ipark2.Ds = pid_out_ds;
	ipark2.Sine = sin_theta;
	ipark2.Cosine = cos_theta;
	IPARK_MACRO(ipark2);
	//svpwm
	svgen2.Ualpha = ipark2.Alpha;
	svgen2.Ubeta  = ipark2.Beta;
	SVGENDQ_MACRO(svgen2);
	
	out_range_min_max(&svgen2.Ta, 1600);
	out_range_min_max(&svgen2.Tb, 1600);
	out_range_min_max(&svgen2.Tc, 1600);
	if((get_over_current_status() !=0) || (get_encoder_state() !=0)){
		motor2_set_pwm(0, 0, 0);
	}
	else{
		motor2_set_pwm(svgen2.Ta +HALF_PWM, svgen2.Tb +HALF_PWM, svgen2.Tc +HALF_PWM);
	}
	//pwm out
}
/**********************************************************/
void run_1khz(void)
{
	static uint16_t count_run =0;
	count_run ++;
	count_run %= 10;
	if(count_run ==0){
		Window_filter_Update_all();
		sys_ticks_time_plus();
		//odom_update();
	}
}
/***************************************************/
void foc_time_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseStructure.TIM_Period 		= 100 - 1;		//--10K HZ
	TIM_TimeBaseStructure.TIM_Prescaler 	= 168/2 -1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);	
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM6, ENABLE);
}
/***************************************************/
void TIM6_DAC_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	foc_level_four();
	foc_level_four2();
	run_1khz();
}

/*===========================end of file====================*/


