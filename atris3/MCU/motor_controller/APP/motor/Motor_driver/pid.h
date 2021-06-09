/******************** (C) COPYRIGHT 2016********************
 * �ļ���  ��main.c
 * ����    ��         
 * ��汾  ��ST3.5.0
*********************************************************/
#ifndef _PID_H_
#define _PID_H_
/*------------------ͷ�ļ�����-----------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <math.h>
/*------------------define---------------------------------------------------------------------------------------*/

#define INTERRUPT_DISABLE __set_PRIMASK(1)  	/*�ر������ж�   */
#define INTERRUPT_ENABLE  __set_PRIMASK(0)  	/*�������ж�   */

/*------------------�ṹ�嶨��----------------------------------------------------------------------------------*/	
typedef struct
{
	float measured;		//< ����ֵ
	float desired;      //< ����������ֵ
	float error;        //< ����ֵ-ʵ��ֵ
	float preMeasure;   //< ǰһ��ƫ��
	float integ;        //< ���ֲ���
	float deriv;        //< ΢�ֲ���
	float kp;           //< ��������
	float ki;           //< ���ֲ���
	float kd;           //< ΢�ֲ���
	float outC;			//< �ٶ�ǰ��
	float outP;         //< pid�������֣�������
	float outI;         //< pid���ֲ��֣�������
	float outD;         //< pid΢�ֲ��֣�������
	float iLimit;       //< ��������
	float outPutMax;
	float outPID;
} pidsuite;

/*------------------��������------------------------------------------------------------------------------------*/
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


