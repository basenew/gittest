
/******************** (C) COPYRIGHT 2016 ********************
* Filename:
* Description:
* Platform:
* Other info:
* Author         Date          Notes
xiangbin.huang   2017.02.21    The first version template
***************************************************************/
#include "electric_angle.h"
#include <stdlib.h>

volatile int32_t First_Angle =0;
volatile float Ele_Angle =0;
volatile int32_t hall_angle =0;

volatile int32_t First_Angle2 =0;
volatile float Ele_Angle2 =0;
volatile int32_t hall_angle2 =0;

extern volatile uint8_t motor1HallState, motor2HallState;
extern volatile uint8_t motor1HallTriger, motor2HallTriger;

volatile uint8_t motor_encoder_erro_status 	= 0;
/*  hall        step    startangle      edgehall    edgeangle
    2(010)      1           30 - 30  <
                                       >     8          30
    6(110)      2           90 - 30  <         
                                       >    10          90
    4(100)      3           150 - 30 <
                                       >    9           150
    5(101)      4           210 - 30 <
                                       >    6           210
    1(001)      5           270 - 30 <
                                       >    4           270
    3(011)      6           330 - 30 <
                                       >    5           330
    2(010)      1           30 - 30  <
                                       >    8           30
    6(110)      2           90 - 30  <         
*/
const int8_t hall_to_step[] = {0,5,1,6,3,4,2};
//const int start_step_to_angle[] = {-1000,0-180,60-180,120-180,180-180,240-180,300-180};
  const int start_step_to_angle[] = {-1000,-180,  -120,  -60,      0,      60,     120};
//const int start_hall_to_angle[] = {-1000,240-180,0-180,300-180,120-180,180-180,60-180};
  const int start_hall_to_angle[] = {-1000,60,     -180,    120,    -60,    0,      -120};
//                                  
const float hall_to_angle[] =    {-1000,//0
                                    -1000,//1
                                    -1000,//2
                                    -1000,//3
                                    90,//270-180,//4
                                    150,//330-180,//5
                                    30,//210-180,//6
                                    -1000,//7
                                    -150,//30-180,//8
                                    -30,//150-180,//9
                                    -90,//90-180,//10
                                    -1000};
struct EncoderErroStru{
	volatile uint8_t 	motor1_encoder_erro_status;
	volatile uint8_t 	motor2_encoder_erro_status;
	volatile int32_t 	ele1_angle_a_cycle_tim_count;
	volatile int32_t 	ele2_angle_a_cycle_tim_count;
	volatile uint8_t 	motor1_hall_state;
	volatile uint8_t 	motor2_hall_state;
	volatile uint8_t 	motor1_hall_count;
	volatile uint8_t 	motor2_hall_count;
};
struct EncoderErroStru encoder_erro_state ={0,0,0,0,0,0,0,0};

uint8_t get_encoder_state(void)
{
	return motor_encoder_erro_status;
}

/**********************************************************/
int16_t hall_electric_angle_4000(void)
{
	static uint8_t old_hall =0;  
    int hall_to_a;    
    if(old_hall == 0){         
       hall_to_a =  start_hall_to_angle[motor1HallState];
    }else{
       hall_to_a =  hall_to_angle[motor1HallState + old_hall]; 
    }
    old_hall = motor1HallState;
    Ele_Angle = hall_to_a;
    return Ele_Angle;
}
/**********************************************************/
float encoder_electric_angle_4000(void)
{
	static int encoder_cnt[2];
	int delta_cnt;
	static uint8_t first_in =1;
	
	if(first_in){
		first_in = 0;
		encoder_cnt[1] = TIM4->CNT;
		encoder_cnt[0] = encoder_cnt[1];
	}
	else{
		encoder_cnt[1] = TIM4->CNT;
		delta_cnt = encoder_cnt[1] - encoder_cnt[0];
		if(delta_cnt > 10000){
		  delta_cnt -= 65535;
		}
		else if(delta_cnt < -10000){
		  delta_cnt += 65535;
		}
		encoder_cnt[0] = encoder_cnt[1];
		Ele_Angle += (delta_cnt* ANGLE_360/MECHANICAL_ANGLE_F);
		encoder_erro_state.ele1_angle_a_cycle_tim_count += delta_cnt;
	}
	if(Ele_Angle >(ANGLE_360/2)){
		Ele_Angle -= ANGLE_360;
	}
	else if(Ele_Angle <(-ANGLE_360/2) ){
		Ele_Angle +=ANGLE_360;
	}
	return  Ele_Angle;
}

/**********************************************************/
int16_t hall2_electric_angle_4000(void)
{
    static uint8_t old_hall =0;  
    int hall_to_a;    
    if(old_hall == 0){         
       hall_to_a =  start_hall_to_angle[motor2HallState];
    }else{
       hall_to_a =  hall_to_angle[motor2HallState + old_hall]; 
    }
    old_hall = motor2HallState;
    Ele_Angle2 = hall_to_a;
    return Ele_Angle2;
}
/**********************************************************/
float encoder2_electric_angle_4000(void)
{
	static int encoder_cnt[2];
	int delta_cnt;
	static uint8_t first_in =1;
	
	if(first_in){
		first_in = 0;
		encoder_cnt[1] = TIM2->CNT;
		encoder_cnt[0] = encoder_cnt[1];
	}
	else{
		encoder_cnt[1] = TIM2->CNT;
		delta_cnt = encoder_cnt[1] - encoder_cnt[0];
		if(delta_cnt > 10000){
		  delta_cnt -= 65535;
		}
		else if(delta_cnt < -10000){
		  delta_cnt += 65535;
		}
		encoder_cnt[0] = encoder_cnt[1];
		Ele_Angle2 += (delta_cnt * ANGLE_360/MECHANICAL_ANGLE_F);
		encoder_erro_state.ele2_angle_a_cycle_tim_count += delta_cnt;
	}
	//
	if(Ele_Angle2 >(ANGLE_360/2)){
		Ele_Angle2 -= ANGLE_360;
	}
	else if(Ele_Angle2 <(-ANGLE_360/2) ){
		Ele_Angle2 +=ANGLE_360;
	}
	return  Ele_Angle2;
}

//------------------------------------------------------------



