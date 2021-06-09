/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：main.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/
#ifndef _PID_H_
#define _PID_H_
/*------------------头文件包含-----------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <math.h>
/*------------------define---------------------------------------------------------------------------------------*/

#define INTERRUPT_DISABLE __set_PRIMASK(1)  	/*关闭所有中断   */
#define INTERRUPT_ENABLE  __set_PRIMASK(0)  	/*打开所有中断   */

/*------------------结构体定义----------------------------------------------------------------------------------*/	
typedef struct
{
	float measured;		//< 测量值
	float desired;      //< 被调量期望值
	float error;        //< 期望值-实际值
	float preMeasure;   //< 前一次偏差
	float integ;        //< 积分部分
	float deriv;        //< 微分部分
	float kp;           //< 比例参数
	float ki;           //< 积分参数
	float kd;           //< 微分参数
	float outC;			//< 速度前馈
	float outP;         //< pid比例部分，调试用
	float outI;         //< pid积分部分，调试用
	float outD;         //< pid微分部分，调试用
	float iLimit;       //< 积分限制
	float outPutMax;
	float outPID;
} pidsuite;

/*------------------函数声明------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void pid_param_init_all(void);
float pid_update(pidsuite* pid, float damp);
void pwm_range_min_max(float *motor, int16_t range);
void pid_set_measured1(float measured);
void pid_set_desired1(float desired );
void pid_set_measured2(float measured);
void pid_set_desired2(float desired );
float pid_motor1_update(void);
float pid_motor2_update(void);
#ifdef __cplusplus
}
#endif
#endif


/*===============================================================================================================*/


