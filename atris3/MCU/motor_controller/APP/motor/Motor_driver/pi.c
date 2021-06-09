/******************** (C) COPYRIGHT 2016********************
 * �ļ���  ��.c
 * ����    ��         
 * ��汾  ��ST3.5.0
*********************************************************/

#include "pi.h"

#define ENCODER_MAX				65535.0f
#define HALF_ENCODER_MAX		32768.0f
#define ERRO_MAX				300.0f
 
const float PI_MOTOR_KP =8.00f;    			//����ϵ��
const float PI_MOTOR_KI =0.00f;				//����ϵ��	
const float PI_MOTOR_KD =20.0f;    			//����ϵ��
const float PI_MOTOR_KV =10.0f;    			//�ٶ�ǰ��
const float PI_MOTOR_KA =0.0f;    			//���ٶ�ǰ��
const float DEFAULT_OUTPUT_MAX_I = 600;		//
const float DEFAULT_OUTPUT_MAX_Q = 1500;	//PID�������

PI_Control motor1_pi, motor2_pi;
/************************************************************/
void pi_init(PI_Control* pi, float measured)       
{
	pi->speed = 0;
	pi->damp = 0;
	pi->measured = measured;
	pi->desired = measured;
	pi->error = 0;
	pi->kp = PI_MOTOR_KP;
	pi->ki = PI_MOTOR_KI;
	pi->kd = PI_MOTOR_KD;
	pi->kv = PI_MOTOR_KV;
//	pi->ka = PI_MOTOR_KA;
//	if(PI_MOTOR_KI >0.001f){
//		pi->integ_limit = DEFAULT_OUTPUT_MAX_I/ PI_MOTOR_KI;
//	}
//	else{
//		pi->integ_limit = 1;
//	}
	pi->outPutMax = DEFAULT_OUTPUT_MAX_Q;
	pi->integ = 0.0f;
}
/************************************************************/



/************************************************************/
float pi_update(PI_Control* pi) //1ms
{
	//��������λ��
	if(-ERRO_MAX < pi->error  &&  pi->error < ERRO_MAX){
		pi->desired += pi->speed * 0.0533333f; //-- 1/1.875f
	}
	if(pi->desired > ENCODER_MAX){
		pi->desired -= ENCODER_MAX;
	}
	else if(pi->desired <0){
		pi->desired += ENCODER_MAX;
	}
	//��������λ����ʵ��λ��ƫ��
	pi->error = pi->desired - pi->measured;			//-- ƫ�����-����ֵ
	if(pi->error > HALF_ENCODER_MAX){
		pi->error -= ENCODER_MAX;
	}
	else if(pi->error < -HALF_ENCODER_MAX){
		pi->error += ENCODER_MAX;
	}
	//��������
//	if(-5 < pi->error && pi->error <5){
//		pi->error = 0;
//	}
	//����
//	if(-pi->integ_limit < pi->outI && pi->outI < pi->integ_limit){
//		pi->integ += pi->error;
//	}
	
	pi->outP = pi->kp * pi->error;					//--P���
//	pi->outI = pi->ki * pi->integ;					//--I���
	pi->outD = pi->kd * pi->damp;					//--D���
	pi->outV = pi->kv * pi->speed;					//--F���
	pi->outPID = pi->outP  +pi->outD + pi->outV;
	
	pi->speed_previous = pi->speed;
	if(pi->outPID > pi->outPutMax){
		return pi->outPutMax;
	}
	else if(pi->outPID < -pi->outPutMax){
		return -pi->outPutMax;
	}
	else{
		return pi->outPID;
	}
}
/************************************************************/
void pi_set_speed(PI_Control *pi, float speed )
{
	pi->speed = speed;
}
/************************************************************/
void pi_set_damp(PI_Control *pi, float damp )
{
	pi->damp = damp;
}

/************************************************************/
void pi_set_measured(PI_Control *pi, float measured )
{
	pi->measured = measured;
}
/************************************************************/
void pi_init_motor1(void)
{
	pi_init(&motor1_pi, TIM4->CNT);
}

/************************************************************/
void pi_set_measured1(float measured )
{
	pi_set_measured(&motor1_pi, measured);
}
/************************************************************/
void pi_set_speed1(float speed )
{
	pi_set_speed(&motor1_pi, speed);
}
/************************************************************/
float pi_update1(void)
{
	return pi_update(&motor1_pi);
}
/************************************************************/
void pi_set_damp1(float damp)
{
	pi_set_damp(&motor1_pi, damp);
}

/************************************************************/
void pi_init_motor2(void)
{
	pi_init(&motor2_pi, TIM2->CNT);
}

/************************************************************/
void pi_set_measured2(float measured )
{
	pi_set_measured(&motor2_pi, measured);
}
/************************************************************/
void pi_set_speed2(float speed )
{
	pi_set_speed(&motor2_pi, speed);
}
/************************************************************/
float pi_update2(void)
{
	return pi_update(&motor2_pi);
}
/************************************************************/
void pi_set_damp2(float damp)
{
	pi_set_damp(&motor2_pi, damp);
}

/************************************************************/
void pi_set_motor_speed(float speed1, float speed2 )
{
	pi_set_speed(&motor1_pi, speed1);
	pi_set_speed(&motor2_pi, speed2);
}




/*===========================end of file====================*/


