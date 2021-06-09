/******************** (C) COPYRIGHT 2016********************
 * �ļ���  ��main.c
 * ����    ��         
 * ��汾  ��ST3.5.0
*********************************************************/
#ifndef _PI_H_
#define _PI_H_
/*------------------ͷ�ļ�����-----------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <math.h>
/*------------------define---------------------------------------------------------------------------------------*/

#define INTERRUPT_DISABLE __set_PRIMASK(1)  	/*�ر������ж�   */
#define INTERRUPT_ENABLE  __set_PRIMASK(0)  	/*�������ж�   */

/*------------------�ṹ�嶨��----------------------------------------------------------------------------------*/	
typedef struct
{
	float speed;			//< ����ٶ�
	float speed_previous;	//< �����һ���ٶ�
	float damp;				//< �������
	float measured;		//< ��ǰ��
	float desired;      //< ������
	float error;        //< ���ֵ
	float integ;        //< ����
	float integ_limit;        //< ����
	float kp;           //< ��������
	float ki;           //< ��������
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

/*------------------��������------------------------------------------------------------------------------------*/
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


