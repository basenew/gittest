/******************** (C) COPYRIGHT 2016********************
 * �ļ���  ��main.c
 * ����    ��         
 * ��汾  ��ST3.5.0
*********************************************************/

#include "pid.h"

/*------------------��������-----------------------------------------------------------------------------------*/

const float PID_MOTOR_KP =150.00f;    	//
const float PID_MOTOR_KI =0.00f;     	//
const float PID_MOTOR_KD =0.00f;     	//

const float DEFAULT_PID_INTEGRATION_LIMIT = 800;	//���ֱ�������
const float DEFAULT_OUTPUT_MAX = 1600;				//PID�������

pidsuite pidMotor1;  
pidsuite pidMotor2; 
	 
/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
void pid_init(pidsuite* pid, float desired, float kp, float ki, float kd)       
{
  pid->error = 0;
  pid->preMeasure = 0;
  pid->integ = 0;
  pid->deriv = 0;
  pid->desired = desired;
  pid->measured= desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->outPutMax = DEFAULT_OUTPUT_MAX;
}
/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
void pid_set(pidsuite* pid,float kp, float ki, float kd)       
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}
/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
float pid_update(pidsuite* pid, float damp)
{
	pid->error 	= pid->desired - pid->measured;		//ƫ�����-����ֵ			 		    
	pid->outP = pid->kp * pid->error;							//P���
	pid->outPID = pid->outP;	         
	if(pid->outPID > pid->outPutMax ){
		pid->outPID = pid->outPutMax;
	}
	else if(pid->outPID < -pid->outPutMax){
		pid->outPID = -pid->outPutMax;
	}
	return pid->outPID;
}
/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
void pid_param_init_all(void)
{
    pid_init(&pidMotor1, 0, PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD);
    pid_init(&pidMotor2, 0, PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD);
}
/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
void pwm_range_min_max(float *motor, int16_t range)
{
  if (*motor <-range){
    *motor = -range;
	}
  if(*motor >range){
    *motor = range;
	}
}
/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
void pid_set_desired1(float desired )
{
	pidMotor1.desired = desired;
}
/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
void pid_set_measured1(float measured)
{
	pidMotor1.measured = measured;
}
/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
void pid_set_desired2(float desired )
{
	pidMotor2.desired = desired;
}
/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
void pid_set_measured2(float measured)
{
	pidMotor2.measured = measured;
}
/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
float pid_motor1_update(void)
{
	return pid_update(&pidMotor1, 0);
}

/*************************************************************
  Function   :
  Description:
  Input      : 
  return     : 
*************************************************************/
float pid_motor2_update(void)
{
	return pid_update(&pidMotor2, 0);
}


/*===========================end of file====================*/


