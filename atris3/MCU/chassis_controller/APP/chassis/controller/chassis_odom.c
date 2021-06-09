
#include <rtthread.h>
#include "chassis_odom.h"
#include "chassis_common.h"
#include "controller.h"
#include "complex.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ee.h"
#include "common.h"
#define LOG_TAG              "odom"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
extern WHEEL_STEER_def  leftFront;
extern WHEEL_STEER_def  rightFront;
extern WHEEL_STEER_def  leftRear;
extern WHEEL_STEER_def  rightRear;

volatile COMPLEX_def chassisPostion;
volatile double chassisPostion_X;
volatile double chassis_abs_odom = 0.0f;
volatile COMPLEX_def chassisOdomTheta;
volatile COMPLEX_def chassisSpeed;
volatile COMPLEX_def chassisSpeedTheta;
void save_abs_odom(double abs_odom);
double  wheelDiffAndTrans( WHEEL_STEER_def  *steer){
    double re;
    int currentEncode =steer->currentDriveEncodeValue & (DF_ENCODER_ODOM -1);
    int diffEncode = currentEncode - steer->backEncodeValue;
    steer->backEncodeValue = currentEncode;
    if(diffEncode > (DF_ENCODER_ODOM /2))
        diffEncode-=DF_ENCODER_ODOM;
    if(diffEncode < (-DF_ENCODER_ODOM /2))
        diffEncode+=DF_ENCODER_ODOM;
    re = steer->driveDirect * diffEncode * DF_WHEEL_DIAMATER * PI;
    re = re / ((double)DF_ENCODER_RESOLUTION) / DF_PLANET_GEAR_REDUCTION_RATIO ;
   // if(steer->driveId == 0x14)
   //     rt_kprintf("%s(%d,%d):speed:%d\r\n",steer->name,diffEncode,currentEncode,(int)(re*1000)); 
    return re;
    
}


double  wheelSpeedTrans( WHEEL_STEER_def  *steer){
    double re;

    re = ((double)steer->driveDirect) * ((double)steer->currentDriveSpeed) * (DF_WHEEL_DIAMATER * PI * DF_HELISHI_RPM_UNIT);
    re = re / DF_MINUTE / DF_PLANET_GEAR_REDUCTION_RATIO;
    return re;
    
}


double  wheelTheta( WHEEL_STEER_def  *steer){
    double re;
    int currentTheta = steer->currentSteering485EncodeValue;
    if(currentTheta < steer->maximumAngle485)
        currentTheta+=DF_ENCODER485_RESOLUTION;
    re = (currentTheta - steer->center485);
    return re * 2.0f * PI / DF_ENCODER485_RESOLUTION;
    
}

double  wheelTheta485( WHEEL_STEER_def  *steer){
    double re;
    int currentTheta = steer->currentSteering485EncodeValue;
    if(currentTheta < steer->maximumAngle485)
        currentTheta+=DF_ENCODER485_RESOLUTION;
    re = currentTheta;
    return re * 2.0f * PI / DF_ENCODER485_RESOLUTION;
    
}


double  wheelSpeed( WHEEL_STEER_def  *steer){
    double re;
    int currentTheta = steer->currentSteering485EncodeValue;
    if(currentTheta < steer->maximumAngle485)
        currentTheta+=DF_ENCODER485_RESOLUTION;
    re = (currentTheta - steer->center485);
    return re * 2.0f * PI / DF_ENCODER485_RESOLUTION;
    
}


void PostionIntegral(void){
    COMPLEX_def  leftFrontP;
    COMPLEX_def  rightFrontP;
    COMPLEX_def  leftRearP;
    COMPLEX_def  rightRearP;
    COMPLEX_def  temP; 
    COMPLEX_def  temTheta; 
    uint32_t s_timer1= 0;
    double th;
    
    COMPLEX_def  tem4;    
    tem4.real = 4.0f;
    tem4.imag = 0.0f;
    //currentDriveEncodeValue
    // rt_kprintf("leftFrontP:th:%d d:%d\r\n",(int)(wheelTheta(&leftFront)*180/PI),(int)(wheelDiffAndTrans(&leftFront) * 10000));

    leftFrontP = radModToComplex(th = wheelTheta(&leftFront), wheelDiffAndTrans(&leftFront));
    rightFrontP = radModToComplex(wheelTheta(&rightFront),wheelDiffAndTrans(&rightFront));
    leftRearP = radModToComplex(wheelTheta(&leftRear),wheelDiffAndTrans(&leftRear));
    rightRearP = radModToComplex(wheelTheta(&rightRear),wheelDiffAndTrans(&rightRear));

   // rt_kprintf("x:%d y:%d ",(int)(leftFrontP.real*10000),(int)(leftFrontP.imag*10000)); 
   // rt_kprintf("\tx:%d y:%d ",(int)(rightFrontP.real*10000),(int)(rightFrontP.imag*10000)); 
   // rt_kprintf("\tx:%d y:%d ",(int)(leftRearP.real*10000),(int)(leftRearP.imag*10000)); 
   // rt_kprintf("\tx:%d y:%d ",(int)(rightRearP.real*10000),(int)(rightRearP.imag*10000)); 
    //return;
    temP = complexAdd(complexAdd(leftFrontP,rightFrontP),complexAdd(leftRearP,rightRearP));
    //temP = complexDiv(temP,tem4);
    //rt_kprintf("\tx:%d y:%d ",(int)(temP.real*10000),(int)(temP.imag*10000)); 
    temP = complexDiv(temP,tem4);
   // rt_kprintf("\tx:%d y:%d \r\n",(int)(temP.real*10000),(int)(temP.imag*10000)); 
    leftFrontP =complexSub(leftFrontP,temP);
    rightFrontP =complexSub(rightFrontP,temP);
    leftRearP =complexSub(leftRearP,temP);
    rightRearP =complexSub(rightRearP,temP);


     leftFrontP =complexDiv(leftFrontP,leftFront.axlePoint);
    rightFrontP =complexDiv(rightFrontP,rightFront.axlePoint);
    leftRearP =complexDiv(leftRearP,leftRear.axlePoint);
    rightRearP =complexDiv(rightRearP,rightRear.axlePoint);   
    th = (leftFrontP.imag + rightFrontP.imag + leftRearP.imag + rightRearP.imag)/ 4.0f;
    //temTheta = complexAdd(complexAdd(leftFrontP,rightFrontP),complexAdd(leftRearP,rightRearP));
    
    chassisOdomTheta.imag += th;
    if(chassisOdomTheta.imag > PI)
        chassisOdomTheta.imag -= (2 * PI);
    if(chassisOdomTheta.imag < (-PI))
        chassisOdomTheta.imag += (2 * PI);
    temTheta = radModToComplex(chassisOdomTheta.imag,1.0f);
    chassisPostion_X+=temP.real;
    chassis_abs_odom+=temP.real;
    chassisPostion = complexAdd(chassisPostion,complexMul(temP,temTheta));
    
    if (os_gettime_ms() - s_timer1 >= ABS_ODOM_SAVE_PERIOD)
        {
            s_timer1 = os_gettime_ms();
            save_abs_odom(chassis_abs_odom);
        }else if(os_gettime_ms() < s_timer1){
             s_timer1 = os_gettime_ms();
        }
    
    
}
void speedIntegral(void){
    COMPLEX_def  leftFrontS;
    COMPLEX_def  rightFrontS;
    COMPLEX_def  leftRearS;
    COMPLEX_def  rightRearS;    
    COMPLEX_def  temS; 
    COMPLEX_def  tem4;
    tem4.real = 4.0f;
    tem4.imag = 0.0f;
    
    leftFrontS = radModToComplex(wheelTheta(&leftFront), wheelSpeedTrans(&leftFront));
    rightFrontS = radModToComplex(wheelTheta(&rightFront),wheelSpeedTrans(&rightFront));
    leftRearS = radModToComplex(wheelTheta(&leftRear),wheelSpeedTrans(&leftRear));
    rightRearS = radModToComplex(wheelTheta(&rightRear),wheelSpeedTrans(&rightRear));
//    rt_kprintf("  c:%d r:%d ",leftFront.controlSpeed,leftFront.currentDriveSpeed); 
//    rt_kprintf("  x:%d y:%d ",(int)(leftFrontS.real*10000),(int)(leftFrontS.imag*10000)); 
//    rt_kprintf("\tx:%d y:%d ",(int)(rightFrontS.real*10000),(int)(rightFrontS.imag*10000)); 
//    rt_kprintf("\tx:%d y:%d ",(int)(leftRearS.real*10000),(int)(leftRearS.imag*10000)); 
//    rt_kprintf("\tx:%d y:%d ",(int)(rightRearS.real*10000),(int)(rightRearS.imag*10000));     

    temS = complexAdd(complexAdd(leftFrontS,rightFrontS),complexAdd(leftRearS,rightRearS));
  //  rt_kprintf("\tx:%d y:%d ",(int)(temS.real*10000),(int)(temS.imag*10000)); 
    chassisSpeed = temS = complexDiv(temS,tem4);    
   // rt_kprintf("\tx:%d y:%d \r\n",(int)(temS.real*10000),(int)(temS.imag*10000)); 
    leftFrontS =complexSub(leftFrontS,temS);
    rightFrontS =complexSub(rightFrontS,temS);
    leftRearS =complexSub(leftRearS,temS);
    rightRearS =complexSub(rightRearS,temS);


    leftFrontS =complexDiv(leftFrontS,leftFront.axlePoint);
    rightFrontS =complexDiv(rightFrontS,rightFront.axlePoint);
    leftRearS =complexDiv(leftRearS,leftRear.axlePoint);
    rightRearS =complexDiv(rightRearS,rightRear.axlePoint);   
    
    //temTheta = complexAdd(complexAdd(leftFrontS,rightFrontS),complexAdd(leftRearS,rightRearS));
    chassisSpeedTheta.imag = (leftFrontS.imag + rightFrontS.imag + leftRearS.imag + rightRearS.imag) / 4.0; //complexDiv(temTheta,tem4);
    

}

void sendOdom(){
    static int inc = 0;
    
    send_odom(chassisPostion,chassisSpeed,chassisOdomTheta.imag,chassisSpeedTheta.imag,chassis_abs_odom);
    inc++;
    if(((inc%1000) == 0) && 1){
        int thetaP = (int)(chassisOdomTheta.imag * 180.0f / PI);
        int thetaS = (int)(chassisSpeedTheta.imag * 180.0f / PI);
//        LOG_I("postion x:%d,y:%d,th:%d \t speed  x:%d,y:%d,th:%d \r\n",
//                                        (int)(chassisPostion.real*1000),
//                                        (int)(chassisPostion.imag*1000),
//                                        thetaP,
//                                        (int)(chassisSpeed.real*1000),
//                                        (int)(chassisSpeed.imag*1000),
//                                        thetaS);
        
    }
    
}

void save_abs_odom(double abs_odom)
{
    
    if(ee_write(EE_ADDR_ODOM_FLAGS, (uint8_t *)&chassis_abs_odom, sizeof(chassis_abs_odom)) != RT_EOK)
       LOG_I("%s abs odom save false\r\n",__FUNCTION__);
}

void init_abs_odom(void)
{
    
    if(ee_read(EE_ADDR_ODOM_FLAGS, (uint8_t *)&chassis_abs_odom, sizeof(chassis_abs_odom)) != RT_EOK){
        LOG_I("%s abs odom READ false\r\n",__FUNCTION__);
        chassis_abs_odom = 0.0f;
    }
}
static void odom_thread_entry(void *_param)
{

    chassisPostion.real = 0;
    chassisPostion.imag = 0;
    
    chassisOdomTheta.real = 0;
    chassisOdomTheta.imag = 0;
    
    chassisSpeed.real = 0;
    chassisSpeed.imag = 0;
    
    chassisSpeedTheta.real = 0;
    chassisSpeedTheta.imag = 0;
    chassisPostion_X = 0.0f;

    leftFront.backEncodeValue = leftFront.currentDriveEncodeValue & (DF_ENCODER_ODOM -1);
    rightFront.backEncodeValue = rightFront.currentDriveEncodeValue & (DF_ENCODER_ODOM -1);
    leftRear.backEncodeValue = leftRear.currentDriveEncodeValue & (DF_ENCODER_ODOM -1);
    rightRear.backEncodeValue = rightRear.currentDriveEncodeValue & (DF_ENCODER_ODOM -1);
    init_abs_odom();
    rt_thread_mdelay(1000*5);  
   while(1){
        PostionIntegral();
        speedIntegral();
        sendOdom();
        rt_thread_mdelay(10);  
   }
    
}



int32_t odom_init(void)
{

    rt_thread_mdelay(10000);
if(DF_THREAD_STATIC_MEMORY == 0){
    rt_thread_t thread = rt_thread_create("odomthread", odom_thread_entry, RT_NULL, TASK_STACK_SIZE_ODOM, TASK_PRIORITY_ODOM, 10);

    LOG_I("odom odom_init\r\n");
    if (thread != RT_NULL) {
        
        rt_thread_startup(thread);
    }
    else{
       
        return -1;
    }
    
}else{
    static struct rt_thread odom_thread;
    ALIGN(RT_ALIGN_SIZE)
    static char odom_thread_stack[TASK_STACK_SIZE_ODOM]; 
    	rt_err_t result = RT_EOK;
    result = rt_thread_init(&odom_thread,
                            "odomthread",
                            odom_thread_entry, RT_NULL,
                            &odom_thread_stack[0], sizeof(odom_thread_stack),
                            TASK_PRIORITY_ODOM, 10);

    if (result == RT_EOK)
    	rt_thread_startup(&odom_thread);
    else
    	LOG_I("odom thread create failed.");
    
}
    return 0;
}
