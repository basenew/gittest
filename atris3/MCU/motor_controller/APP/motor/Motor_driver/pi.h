/******************** (C) COPYRIGHT 2016********************
 * 文件名  ：main.c
 * 描述    ：         
 * 库版本  ：ST3.5.0
*********************************************************/
#ifndef _PI_H_
#define _PI_H_
/*------------------头文件包含-----------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <math.h>
/*------------------define---------------------------------------------------------------------------------------*/

#define INTERRUPT_DISABLE __set_PRIMASK(1)  	/*关闭所有中断   */
#define INTERRUPT_ENABLE  __set_PRIMASK(0)  	/*打开所有中断   */

/*------------------结构体定义----------------------------------------------------------------------------------*/	
typedef struct
{
	float speed;			//< 电机速度
	float speed_previous;	//< 电机上一个速度
	float damp;				//< 电机阻尼
	float measured;		//< 当前量
	float desired;      //< 期望量
	float error;        //< 误差值
	float integ;        //< 积分
	float integ_limit;        //< 积分
	float kp;           //< 比例参数
	float ki;           //< 比例参数
	float kd;           //<
	float kv;           //<
	float ka;
	float outP;         //< pid
	float outI;         //< pid
	float outD;         //< pid
	float outV;         //< pid
	float outA;         //< pid
	float outPutMax;
	float outPID;
} PI_Control;

/*------------------函数声明------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
void pi_init_motor1(void);
void pi_set_measured1(float measured );
void pi_set_speed1(float speed );
float pi_update1(void);
void pi_set_damp1(float damp);


void pi_init_motor2(void);
void pi_set_measured2(float measured );
void pi_set_speed2(float speed );
float pi_update2(void);
void pi_set_damp2(float damp);

void pi_set_motor_speed(float speed1, float speed2 );
#ifdef __cplusplus
}
#endif

#endif


/*===============================================================================================================*/


