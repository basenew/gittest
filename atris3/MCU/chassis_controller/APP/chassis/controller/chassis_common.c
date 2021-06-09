
#include <rtthread.h>
#include "chassis_common.h"
#include <easyflash.h>
#include "can2.h"

#include "can1_fifo_tx.h"
//#include "can1_fifo_rx.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "can2_fifo_rx.h"
#include "power_ctrl.h"
#define LOG_TAG              "ch_cm"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
WHEEL_STEER_def  leftFront;
WHEEL_STEER_def  rightFront;
WHEEL_STEER_def  leftRear;
WHEEL_STEER_def  rightRear;
volatile int motor_default_id_;
/*shawn : base on the zero center calibration value afer EOL and current position to get the offset
          then over to the zero center postion value after got at eol*/
void oneSteeringZeroFilter(WHEEL_STEER_def  *steer)
{
    double currentRealAngle;
    double zero485 = (double)(steer->maximumAngle485 + steer->minimumAngle485) / 2;
    double angMotor;
    double current485 = steer->currentSteering485EncodeValue;

   //angMotor= angI + axle->centerI
    if(current485 < steer->maximumAngle485)
        current485+=DF_ENCODER485_RESOLUTION;
    currentRealAngle = (current485 - zero485);
    currentRealAngle = currentRealAngle * DF_TIMING_BELT_REDUCTION_RATIO * DF_PLANET_GEAR_REDUCTION_RATIO * DF_ENCODER_RESOLUTION; 

    angMotor = steer->steeringDirect * currentRealAngle / ((double)DF_ENCODER485_RESOLUTION); 
    //steer->centerI = steer->currentSteeringEncodeValue - ((int)angMotor);
    steer->integral_centerI = steer->integral_centerI * (DF_CENTER_I_FILTER_CONSTANT - 1)/DF_CENTER_I_FILTER_CONSTANT;
    steer->integral_centerI = steer->integral_centerI + steer->currentSteeringEncodeValue - ((int)angMotor);
    steer->centerI = steer->integral_centerI / DF_CENTER_I_FILTER_CONSTANT;
 
}


void setOneSteeringZero(WHEEL_STEER_def  *steer)
{
    double currentRealAngle;
    double zero485 = (double)(steer->maximumAngle485 + steer->minimumAngle485) / 2;
    double angMotor;
    double current485 = steer->currentSteering485EncodeValue;

   //angMotor= angI + axle->centerI
    if(current485 < steer->maximumAngle485)
        current485+=DF_ENCODER485_RESOLUTION;
    currentRealAngle = (current485 - zero485);
    currentRealAngle = currentRealAngle * DF_TIMING_BELT_REDUCTION_RATIO * DF_PLANET_GEAR_REDUCTION_RATIO * DF_ENCODER_RESOLUTION; 
    /*shawn: angMotor is the distance rad to the center of abs485*/
    angMotor = steer->steeringDirect * currentRealAngle / ((double)DF_ENCODER485_RESOLUTION); 
		/*shawn: bugs? after got the offset from the abs485,why it is trans to increase encoder. just move to the abs485 center value whit the distance.*/
    steer->centerI = steer->currentSteeringEncodeValue - ((int)angMotor);
    steer->integral_centerI = steer->centerI * DF_CENTER_I_FILTER_CONSTANT;
    steer->center485 =  zero485;   
    LOG_I("%s zero485:%d rad:%d cenI:%d,encode485:%d\r\n",steer->name,(int)zero485,(int)(currentRealAngle *180/PI),steer->centerI,(int)current485);
    
}
void initSteeringZero(void)
{
    char *p;
    char *p1;
    p = ef_get_env(DF_STEER_ZERO_VALUE);
    LOG_I("steer zero value:%s\r\n",p);
    rt_thread_mdelay(1000);  
    if(!strstr(p,"NULL")){

//--------------------------------       
        p1 =p;
        p = strchr(p,',');
        *p = 0;
        p++;
        leftFront.maximumAngle485 = atol(p1);

        p1 =p;
        p = strchr(p,',');
        *p = 0;
        p++;
        leftFront.minimumAngle485 = atol(p1); 
        if(leftFront.minimumAngle485 < leftFront.maximumAngle485)
            leftFront.minimumAngle485+=DF_ENCODER485_RESOLUTION;
        LOG_I("1 %s :%d\r\n",p1,leftFront.maximumAngle485);  
        LOG_I("1 %s :%d\r\n",p1,leftFront.minimumAngle485);       
//--------------------------------       
        p1 =p;
        p = strchr(p,',');
        *p = 0;
        p++;
        rightFront.maximumAngle485 = atol(p1);
        p1 =p;
        p = strchr(p,',');
        *p = 0;
        p++;
        rightFront.minimumAngle485 = atol(p1); 
        if(rightFront.minimumAngle485 < rightFront.maximumAngle485)
            rightFront.minimumAngle485+=DF_ENCODER485_RESOLUTION;
        LOG_I("2 %s :%d\r\n",p1,rightFront.maximumAngle485);
        LOG_I("2 %s :%d\r\n",p1,rightFront.minimumAngle485);  
//--------------------------------       
        p1 =p;
        p = strchr(p,',');
        *p = 0;
        p++;
        leftRear.maximumAngle485 = atol(p1);
        p1 =p;
        p = strchr(p,',');
        *p = 0;
        p++;
        leftRear.minimumAngle485 = atol(p1); 
        if(leftRear.minimumAngle485 < leftRear.maximumAngle485)
            leftRear.minimumAngle485+=DF_ENCODER485_RESOLUTION;
        LOG_I("2 %s :%d\r\n",p1,leftRear.maximumAngle485);
        LOG_I("2 %s :%d\r\n",p1,leftRear.minimumAngle485);  
//--------------------------------       
        p1 =p;
        p = strchr(p,',');
        *p = 0;
        p++;
        rightRear.maximumAngle485 = atol(p1);
        p1 =p;
        p = strchr(p,',');
        *p = 0;
        p++;
        rightRear.minimumAngle485 = atol(p1); 
        if(rightRear.minimumAngle485 < rightRear.maximumAngle485)
            rightRear.minimumAngle485+=DF_ENCODER485_RESOLUTION;
        LOG_I("2 %s :%d\r\n",p1,rightRear.maximumAngle485);
        LOG_I("2 %s :%d\r\n",p1,rightRear.minimumAngle485);  

        LOG_I("read%s\t\t%s\t\t%s\t\t%s\r\n",leftFront.name,rightFront.name,leftRear.name,rightRear.name);
        LOG_I("\t%angle encode\t%angle encode\t%angle encode\t%angle encode\r\n");
        LOG_I("\t%d,%d,%ld,%ld",leftFront.maximumAngle485 * 360/16384,leftFront.minimumAngle485 * 360/16384,leftFront.maximumAngle485,leftFront.minimumAngle485);
        LOG_I(",%d,%d,%ld,%ld",rightFront.maximumAngle485 * 360/16384,rightFront.minimumAngle485 * 360/16384,rightFront.maximumAngle485,rightFront.minimumAngle485);
        LOG_I(",%d,%d,%ld,%ld",leftRear.maximumAngle485 * 360/16384,leftRear.minimumAngle485 * 360/16384,leftRear.maximumAngle485,leftRear.minimumAngle485);
        LOG_I(",%d,%d,%ld,%ld\r\n",rightRear.maximumAngle485 * 360/16384,rightRear.minimumAngle485 * 360/16384,rightRear.maximumAngle485,rightRear.minimumAngle485);
        rt_thread_mdelay(3000);
        
     
        setOneSteeringZero(&leftFront);
        setOneSteeringZero(&rightFront);
        setOneSteeringZero(&leftRear);
        setOneSteeringZero(&rightRear);

        
    }   
    
}


void wheel_steering_Init(void){
   

    strcpy(leftFront.name,"leftFront");
    leftFront.steeringId = DF_LEFT_FRONT_STEERING_ID;
    leftFront.driveId    = DF_LEFT_FRONT_DRIVE_ID;
    leftFront.minimumAngle = DF_STEERING_ENPTY;
    leftFront.maximumAngle = DF_STEERING_ENPTY;
    leftFront.steeringDirect = DF_LEFT_FRONT_STEERING_DIRECT;
    leftFront.driveDirect    = DF_LEFT_FRONT_DRIVE_DIRECT;
    leftFront.axlePoint.real = DF_WHELL_BASE / 2;
    leftFront.axlePoint.imag = DF_TRACK_WIDTH / 2;
    leftFront.steering_power_pin = POWER_24V_LEFT_FRONT_STEER;
    leftFront.drive_power_pin = POWER_24V_LEFT_FRONT_DIRECT;
    leftFront.abs_angle_error = RT_FALSE;
    
    strcpy(rightFront.name,"rightFront");    
    rightFront.steeringId = DF_RIGHT_FRONT_STEERING_ID;
    rightFront.driveId    = DF_RIGHT_FRONT_DRIVE_ID;
    rightFront.minimumAngle = DF_STEERING_ENPTY;
    rightFront.maximumAngle = DF_STEERING_ENPTY;
    rightFront.steeringDirect = DF_RIGHT_FRONT_STEERING_DIRECT;
    rightFront.driveDirect    = DF_RIGHT_FRONT_DRIVE_DIRECT;
    rightFront.axlePoint.real = DF_WHELL_BASE / 2;
    rightFront.axlePoint.imag = -DF_TRACK_WIDTH / 2; 
    rightFront.steering_power_pin = POWER_24V_RIGHT_FRONT_STEER;
    rightFront.drive_power_pin = POWER_24V_RIGHT_FRONT_DIRECT;
    rightFront.abs_angle_error = RT_FALSE;    
    
    strcpy(leftRear.name,"leftRear");    
    leftRear.steeringId = DF_LEFT_REAR_STEERING_ID;
    leftRear.driveId    = DF_LEFT_REAR_DRIVE_ID;
    leftRear.minimumAngle = DF_STEERING_ENPTY;
    leftRear.maximumAngle = DF_STEERING_ENPTY;
    leftRear.steeringDirect = DF_LEFT_REAR_STEERING_DIRECT;
    leftRear.driveDirect    = DF_LEFT_REAR_DRIVE_DIRECT;
    leftRear.axlePoint.real = -DF_WHELL_BASE / 2;
    leftRear.axlePoint.imag = DF_TRACK_WIDTH / 2;
    leftRear.steering_power_pin = POWER_24V_LEFT_TEAR_STEER;
    leftRear.drive_power_pin = POWER_24V_LEFT_TEAR_DIRECT;
    leftRear.abs_angle_error = RT_FALSE;
    
    strcpy(rightRear.name,"rightRear");    
    rightRear.steeringId = DF_RIGHT_REAR_STEERING_ID;
    rightRear.driveId    = DF_RIGHT_REAR_DRIVE_ID;
    rightRear.minimumAngle = DF_STEERING_ENPTY;
    rightRear.maximumAngle = DF_STEERING_ENPTY;
    rightRear.steeringDirect = DF_RIGHT_REAR_STEERING_DIRECT;
    rightRear.driveDirect    = DF_RIGHT_REAR_DRIVE_DIRECT;
    rightRear.axlePoint.real = -DF_WHELL_BASE / 2;
    rightRear.axlePoint.imag = -DF_TRACK_WIDTH / 2;
    rightRear.steering_power_pin = POWER_24V_RIGHT_TEAR_STEER;
    rightRear.drive_power_pin = POWER_24V_RIGHT_TEAR_DIRECT;
    rightRear.abs_angle_error = RT_FALSE;

	CAN_init();	
	can2_fifo_rx_init();
	
    leftFront.init_error_code = CANopen_PP_Init(leftFront.steeringId);
    leftFront.init_error_code |= (CANopen_PV_Init(leftFront.driveId) << 1);
    LOG_I("leftFront.init_error_code:%x",leftFront.init_error_code);
    rightFront.init_error_code = CANopen_PP_Init(rightFront.steeringId);
    rightFront.init_error_code |= (CANopen_PV_Init(rightFront.driveId) << 1);
    LOG_I("rightFront.init_error_code:%x",rightFront.init_error_code);
    leftRear.init_error_code = CANopen_PP_Init(leftRear.steeringId);
    leftRear.init_error_code |= (CANopen_PV_Init(leftRear.driveId) << 1);
    LOG_I("leftRear.init_error_code:%x",leftRear.init_error_code);
    rightRear.init_error_code = CANopen_PP_Init(rightRear.steeringId);
    rightRear.init_error_code |= (CANopen_PV_Init(rightRear.driveId) << 1);
    LOG_I("rightRear.init_error_code:%x",rightRear.init_error_code);
    
   leftFront.init_error_code |= (Motor_Read_Position_Speed(leftFront.steeringId) << 2);    
   leftFront.init_error_code |= (Motor_Read_Speed(leftFront.driveId) << 3);
   LOG_I("leftFront.init_error_code:%x",leftFront.init_error_code);
   rightFront.init_error_code |= (Motor_Read_Position_Speed(rightFront.steeringId) << 2);  
   rightFront.init_error_code |= (Motor_Read_Speed(rightFront.driveId) << 3);
   LOG_I("rightFront.init_error_code:%x",rightFront.init_error_code);
   leftRear.init_error_code |= (Motor_Read_Position_Speed(leftRear.steeringId) << 2);  
   leftRear.init_error_code |= (Motor_Read_Speed(leftRear.driveId) << 3);
   LOG_I("leftRear.init_error_code:%x",leftRear.init_error_code);
   rightRear.init_error_code |= (Motor_Read_Position_Speed(rightRear.steeringId) << 2);  
   rightRear.init_error_code |= (Motor_Read_Speed(rightRear.driveId) << 3);
   LOG_I("rightRear.init_error_code:%x",rightRear.init_error_code);
 

    leftFront.steering_version_year = CANopen_get_version_year(leftFront.steeringId);
    leftFront.steering_version_date = CANopen_get_version_date(leftFront.steeringId);
    leftFront.drive_version_year = CANopen_get_version_year(leftFront.driveId);
    leftFront.drive_version_date = CANopen_get_version_date(leftFront.driveId);

    rightFront.steering_version_year = CANopen_get_version_year(rightFront.steeringId);
    rightFront.steering_version_date = CANopen_get_version_date(rightFront.steeringId);
    rightFront.drive_version_year = CANopen_get_version_year(rightFront.driveId);
    rightFront.drive_version_date = CANopen_get_version_date(rightFront.driveId);

    leftRear.steering_version_year = CANopen_get_version_year(leftRear.steeringId);
    leftRear.steering_version_date = CANopen_get_version_date(leftRear.steeringId);
    leftRear.drive_version_year = CANopen_get_version_year(leftRear.driveId);
    leftRear.drive_version_date = CANopen_get_version_date(leftRear.driveId);

    rightRear.steering_version_year = CANopen_get_version_year(rightRear.steeringId);
    rightRear.steering_version_date = CANopen_get_version_date(rightRear.steeringId);
    rightRear.drive_version_year = CANopen_get_version_year(rightRear.driveId);
    rightRear.drive_version_date = CANopen_get_version_date(rightRear.driveId);

   
   initSteeringZero();
  
	
}

void paraInitSteeringZero(WHEEL_STEER_def  *steer,int *position,int *inc){
    
    *position = steer->currentSteeringEncodeValue;
    *inc = -DF_INIT_ZERO_INC; 
   // Motor_PP_SetEncodeValue(steer->steeringId,*position);
    
}

volatile int calibration_progress = 0;
int wait = 0;
int getCalibrationProgress(void)
{    int re = calibration_progress;
     if(wait == 0xf)
         re = 100;
    return re;
}

rt_bool_t dealSteeringZero(WHEEL_STEER_def  *steer,int *position,int *inc){
    if((steer->minimumAngle == DF_STEERING_ENPTY) || (steer->currentSteeringEncodeValue >= 0))
        (*position)+=(*inc);
    //rt_kprintf("position:%d torque:%d\r\n",*position,steer->currentSteeringtorque);
    if((steer->minimumAngle == DF_STEERING_ENPTY) || (steer->maximumAngle == DF_STEERING_ENPTY))
            CANopen_PP_Set_RUN(steer->steeringId,(*position),DF_INIT_ZERO_SPEED); 
    
    if(steer->currentSteeringtorque > DF_INIT_ZERO_TORQUE){
       // rt_kprintf("position:%d torque:%d\r\n",*position,steer->currentSteeringtorque);
            if(((*inc) == (-DF_INIT_ZERO_INC)) && (steer->minimumAngle == DF_STEERING_ENPTY)){
                steer->minimumAngle = steer->currentSteeringEncodeValue;
                steer->minimumAngle485 = steer->currentSteering485EncodeValue;
                (*inc) = DF_INIT_ZERO_INC;
               (*position) = DF_INIT_ZERO_INC;
                CANopen_PP_Set_RUN(steer->steeringId,DF_INIT_ZERO_INC,DF_INIT_ZERO_SPEED); 
                LOG_I("%s minimumAngle:%d 485:%d\r\n",steer->name,steer->currentSteeringEncodeValue,steer->currentSteering485EncodeValue);
                calibration_progress+=8;
            }
            if(((*inc) == (DF_INIT_ZERO_INC)) && (steer->maximumAngle == DF_STEERING_ENPTY) && (steer->currentSteeringEncodeValue > 0)){
							/*shawn: why it is use the increase encoder and abs485 encoder to calcualte two zero center point
							          why the centerI using the increase encoder? what is effect?*/
                steer->maximumAngle = steer->currentSteeringEncodeValue;
                 steer->maximumAngle485 = steer->currentSteering485EncodeValue;
                LOG_I("%s maximumAngle:%d 485:%d\r\n",steer->name,steer->currentSteeringEncodeValue,steer->currentSteering485EncodeValue);
                (*inc) = 0;
                steer->centerI = (steer->maximumAngle + steer->minimumAngle)/2;
							/*shawn : move the distance by position control.all move relative to the motor controller relative position? not relative to the ABS485???*/
                CANopen_PP_Set_RUN(steer->steeringId,(steer->maximumAngle + steer->minimumAngle)/2,DF_INIT_ZERO_SPEED);
                calibration_progress+=8;
            }  
            
        }  

    if((steer->minimumAngle != DF_STEERING_ENPTY) && 
        (steer->maximumAngle != DF_STEERING_ENPTY) && 
        (ABS(steer->currentSteeringEncodeValue - ((steer->maximumAngle + steer->minimumAngle)/2)) < DF_INIT_ZERO_DEAL)  ) {
        // rt_kprintf("%s  485:  max:%d min:%d\r\n",steer->name,leftFront.maximumAngle485,leftFront.minimumAngle485);
          //  rt_kprintf("to zero\r\n");
          return RT_TRUE;
        }
    return RT_FALSE;
}
void waitSteeringToCentre(WHEEL_STEER_def  *steer){

    //steer->currentSteeringEncodeValue = 0;
    while(steer->currentDriveEncodeValue != 0){Motor_PP_SetEncodeValue(steer->steeringId,0);
        rt_thread_mdelay(50);
    }


}
void SteeringMoveSetp(WHEEL_STEER_def  *steer,float rad)
{
    double ang = rad * DF_TIMING_BELT_REDUCTION_RATIO * DF_PLANET_GEAR_REDUCTION_RATIO * DF_ENCODER_RESOLUTION/(PI * 2);
    
    CANopen_PP_Set_RUN(steer->steeringId,steer->currentSteeringEncodeValue - (int)ang,10000);

    
}
/*shawn: why it is steering control by speed mode. what is for? */
void SteeringMoveSpeed(WHEEL_STEER_def  *steer,float speed)
{
    double sp = speed * DF_MINUTE * DF_PLANET_GEAR_REDUCTION_RATIO / (DF_WHEEL_DIAMATER * PI * DF_HELISHI_RPM_UNIT);
    LOG_I("steering zero :%d\r\n",(int)sp);
    CANopen_PV_SET(steer->driveId, 200,200,steer->currentDriveSpeed + (int)(sp * steer->driveDirect));


    
}
rt_bool_t calibrationSteering(void){
  

    int lf_p;   
    int rf_p;   
    int lr_p;   
    int rr_p;   
    int lf_inc; 
    int rf_inc; 
    int lr_inc;
    int rr_inc; 
    char tem[64];
    int waitOld = 0;
    wait = 0;
    calibration_progress = 0;    
   // rt_kprintf("steer zero value:%s\r\n",ef_get_env(DF_STEER_ZERO_VALUE));
    
   LOG_I("start currentSteeringEncodeValue:%.6d\t%.6d\t%.6d\t%.6d\t\r\n",leftFront.currentSteeringEncodeValue
                                                        ,rightFront.currentSteeringEncodeValue
                                                        ,leftRear.currentSteeringEncodeValue
                                                        ,rightRear.currentSteeringEncodeValue);
   LOG_I("start centerI:%.6d\t%.6d\t%.6d\t%.6d\t\r\n",leftFront.centerI
                                                        ,rightFront.centerI
                                                        ,leftRear.centerI
                                                        ,rightRear.centerI); 
    
    
    rt_thread_mdelay(1000);
    paraInitSteeringZero(&leftFront,&lf_p,&lf_inc);
    paraInitSteeringZero(&rightFront,&rf_p,&rf_inc);
    paraInitSteeringZero(&leftRear,&lr_p,&lr_inc);
    paraInitSteeringZero(&rightRear,&rr_p,&rr_inc);
    

    while(wait != 0xf && 1){
        if(dealSteeringZero(&leftRear,&lr_p,&lr_inc)==RT_TRUE)
            wait|=4;
        if(dealSteeringZero(&rightRear,&rr_p,&rr_inc)==RT_TRUE)
           wait|=8;        
        if(dealSteeringZero(&leftFront,&lf_p,&lf_inc)==RT_TRUE)
            wait|=1;
        if(dealSteeringZero(&rightFront,&rf_p,&rf_inc)==RT_TRUE)
            wait|=2;

        if(wait !=waitOld){
            calibration_progress+=8;
            waitOld = wait;
            LOG_I("wait:%x\r\n",wait);
        }
        rt_thread_mdelay(5);

    }
    rt_thread_mdelay(1000);
    LOG_I("steering zero OK\r\n");


    LOG_I("\t%s\t\t%s\t\t%s\t\t%s\r\n",leftFront.name,rightFront.name,leftRear.name,rightRear.name);
    LOG_I("\t%angle encode\t%angle encode\t%angle encode\t%angle encode\r\n");
    LOG_I("\t%d,%d,%d,%d",leftFront.maximumAngle485 * 360/16384,leftFront.minimumAngle485 * 360/16384,leftFront.maximumAngle485,leftFront.minimumAngle485);
    LOG_I(",%d,%d,%d,%d",rightFront.maximumAngle485 * 360/16384,rightFront.minimumAngle485 * 360/16384,rightFront.maximumAngle485,rightFront.minimumAngle485);
    LOG_I(",%d,%d,%d,%d",leftRear.maximumAngle485 * 360/16384,leftRear.minimumAngle485 * 360/16384,leftRear.maximumAngle485,leftRear.minimumAngle485);
    LOG_I(",%d,%d,%d,%d\r\n",rightRear.maximumAngle485 * 360/16384,rightRear.minimumAngle485 * 360/16384,rightRear.maximumAngle485,rightRear.minimumAngle485);
    rt_snprintf(tem,sizeof(tem),"%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",leftFront.maximumAngle485,leftFront.minimumAngle485
                                                         ,rightFront.maximumAngle485,rightFront.minimumAngle485
                                                         ,leftRear.maximumAngle485,leftRear.minimumAngle485
                                                         ,rightRear.maximumAngle485,rightRear.minimumAngle485);
    LOG_I("currentSteeringEncodeValue:%.6d\t%.6d\t%.6d\t%.6d\t\r\n",leftFront.currentSteeringEncodeValue
                                                        ,rightFront.currentSteeringEncodeValue
                                                        ,leftRear.currentSteeringEncodeValue
                                                        ,rightRear.currentSteeringEncodeValue);
     LOG_I("centerI:%.6d\t%.6d\t%.6d\t%.6d\t\r\n",leftFront.centerI
                                                        ,rightFront.centerI
                                                        ,leftRear.centerI
                                                        ,rightRear.centerI);   
    LOG_I("steering zero :%s\r\n",tem);
    ef_set_and_save_env(DF_STEER_ZERO_VALUE,tem);
    rt_thread_mdelay(1000);     
    initSteeringZero();
    
    LOG_I("end currentSteeringEncodeValue:%.6d\t%.6d\t%.6d\t%.6d\t\r\n",leftFront.currentSteeringEncodeValue
                                                        ,rightFront.currentSteeringEncodeValue
                                                        ,leftRear.currentSteeringEncodeValue
                                                        ,rightRear.currentSteeringEncodeValue);
     LOG_I("end centerI:%.6d\t%.6d\t%.6d\t%.6d\t\r\n",leftFront.centerI
                                                        ,rightFront.centerI
                                                        ,leftRear.centerI
                                                        ,rightRear.centerI);   
     return RT_TRUE;
}

void dealReceiveCanMsg(WHEEL_STEER_def  *axle,CanRxMsg msg){
       if(msg.StdId == (DF_STEERING_TPDO_POSITION_TORQUE + axle->steeringId) ){            
            int *po = (int *)&msg.Data[0];
            int *tor = (int *)&msg.Data[4];
            axle->currentSteeringtorque = *tor;
            axle->currentSteeringEncodeValue = *po;
           
            }
            if(msg.StdId == (DF_STEERING_TPDO_485ENCODE + axle->steeringId)){ 
                u16 *error_code = (u16 *)&msg.Data[0];
               u16 *temperature = (u16 *)&msg.Data[4];
                u16 *en485 = (u16 *)&msg.Data[6];  
                int en4;
#if DF_MOTOR_PRODUCTER == DF_ZHOUNHANXING_MOTOR
    axle->currentSteering485EncodeValue = *en485;//DF_ENCODER485_RESOLUTION - 
#else
    axle->currentSteering485EncodeValue = DF_ENCODER485_RESOLUTION - *en485;//  
#endif
                axle->steering_error_code = *error_code;
                axle->steering_motor_temperature = (s8)*temperature;
                axle->currentSteering485EncodeValue = *en485;//DF_ENCODER485_RESOLUTION - 
                en4 = axle->currentSteering485EncodeValue;
                if(en4 < axle->maximumAngle485)
                    en4+=DF_ENCODER485_RESOLUTION;
               double cen = (axle->maximumAngle485 + axle->minimumAngle485)/2;

            }
            
            if(msg.StdId == (DF_DRIVE_TPDO_ENCODE_SPEED + axle->driveId)){ 
                int *encode = (int *)&msg.Data[0];
                int *speed = (int *)&msg.Data[4];    
                axle->currentDriveEncodeValue = *encode;
                axle->currentDriveSpeed = *speed;
            }
            if(msg.StdId == (DF_DRIVE_TPDO_ERROR_CODE_485 + axle->driveId)){ 
                u16 *error_code = (u16 *)&msg.Data[0];
               u16 *temperature = (u16 *)&msg.Data[2];
                int *tor = (int *)&msg.Data[4];    
                axle->drive_error_code = *error_code;
                axle->drive_motor_temperature =(s8) *temperature;
                axle->currentDrivetorque = *tor;
            }            
}

void setDefaultId(void){
    motor_default_id_ = -1;
}
int getDefaultId(void){
   return motor_default_id_;
}
void dealCanRx(CanRxMsg msg){
    
    if(motor_default_id_ == -1)
        motor_default_id_ = msg.StdId & 0xff;
    if(0)
        rt_kprintf("dealCanRx:%4x(%d):%.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x \r\n",msg.StdId,msg.DLC,
                                    msg.Data[0],
                                    msg.Data[1],
                                    msg.Data[2],
                                    msg.Data[3],
                                    msg.Data[4],
                                    msg.Data[5],
                                    msg.Data[6],
                                    msg.Data[7]
                                    ); 
    dealReceiveCanMsg(&leftFront,msg);
    dealReceiveCanMsg(&rightFront,msg);
    dealReceiveCanMsg(&leftRear,msg);
    dealReceiveCanMsg(&rightRear,msg);
    
}


/* shawn: calculate the motor speed by the Vx, Vy (speed), W(angs) from the navigation.
Vx: vx+WR*cos

*/

rt_bool_t axleRun(WHEEL_STEER_def  *axle,COMPLEX_def speed, COMPLEX_def angS){
    
    rt_bool_t re = RT_TRUE;
     COMPLEX_def temC;
	/* shawn:temC: is the angle speed to vx, Vy, WR*cos */
     temC = complexMul(axle->axlePoint,angS);
    speed = complexAdd(temC,speed);
	/*shawn: rad is the speed vector for the motor 
	         rad is the steering motor direction*/
    float rad = getComplexrad(speed) * axle->steeringDirect ;
	/* shawn: line speed the modu vale, is the */
    double lineV = getComplexMod(speed);
     if(lineV > 50.0f)
        re = RT_FALSE;
   // rt_kprintf("lineV:%d \r\n",(int)(lineV*10000));
    int steerDirect = axle->driveDirect;
    while(rad < DF_MINIMUM_STEERING_ANGLE){rad+=PI;steerDirect = -steerDirect;}
    while(rad > DF_MAXIMUM_STEERING_ANGLE){rad-=PI;steerDirect = -steerDirect;}
    /*shawn: mapping the speed from the wheel to the motor base on the wheel radius and decessratation box*/
    double ang = rad * DF_TIMING_BELT_REDUCTION_RATIO * DF_PLANET_GEAR_REDUCTION_RATIO * DF_ENCODER_RESOLUTION/(PI * 2);
    lineV =  lineV * DF_MINUTE * DF_PLANET_GEAR_REDUCTION_RATIO / (DF_WHEEL_DIAMATER * PI * DF_HELISHI_RPM_UNIT);
    int vI = (int)lineV;
    int angI = (int)ang;
		/*shawn: what is meaning!*/
		/*shawn: jiao speed is the virtue part, so the real part is 0. it is normal .if the real part is not zero is not normal. the line speed will be zero*/
    if(angS.real == 0)
        axle->controlSpeed = vI * steerDirect;
    else axle->controlSpeed = 0.0f;
   // rt_kprintf("rad:%d re:%d\r\n",(int)(rad*100),(int)(getsteeringAngleF(axle)*100));
		/*shawn: this is the bug, the steering angle is base on the increase coder. if the increase coder has the accumulate error, the angle is wrong.
		         it should be base on the RS485 to get the position 
		         the increamental coder has lose the plus to casue the error*/
		/*shawn: what is the travel motor. why it is <1.5 then set the steering motor run */
		/*shawn: first steering then travel*/
		/*zhi xing bai ba zi :  */
    if(angS.real < DF_RELEASE_TRAVEL_MOTOR)
        CANopen_PP_Set_RUN(axle->steeringId,angI + axle->centerI,10000);
    if(!re)
        return RT_FALSE;
    double encode_angle = getsteeringAngleF(axle);
    double abs485_angle = getsteering485AngleF(axle);
    

    /*what it DF_ENCODE_485_ERROR_ANGLE? what the value come from?
		hardware requirement: 485 and encoder difference */
    if(fabs(encode_angle - abs485_angle) < DF_ENCODE_485_ERROR_ANGLE){
        axle->abs_angle_error = RT_FALSE;
       return RT_TRUE;
    }else{
        axle->abs_angle_error = RT_TRUE;

    }        
       /*steering motor position yu target torlerance then  it is ok , then xingjing*/ 
     if(fabs(encode_angle - rad) < 0.2)
       return RT_TRUE;   
    
        //CANopen_PV_SET( axle->driveId, 2000,2000,vI * steerDirect);
    
    return RT_FALSE;
    
}




double getsteeringAngleF(WHEEL_STEER_def  *axle){
    double re = axle->currentSteeringEncodeValue - axle->centerI;
    re = re * PI * 2/(DF_TIMING_BELT_REDUCTION_RATIO * DF_PLANET_GEAR_REDUCTION_RATIO * DF_ENCODER_RESOLUTION);
    return re;
}

double getsteering485AngleF(WHEEL_STEER_def  *axle){
    double re;
    int currentTheta = axle->currentSteering485EncodeValue;
    if(currentTheta < axle->maximumAngle485)
        currentTheta+=DF_ENCODER485_RESOLUTION;
    re = currentTheta;
    return re * 2.0f * PI / DF_ENCODER485_RESOLUTION;
}
