

#ifndef __CHASSIS_ODOM_H
#define __CHASSIS_ODOM_H	 

//////////////////////////////////////////////////////////////////////////////////	 
#ifdef __cplusplus
 extern "C" {
#endif																    
#include "complex.h"
#include "chassis_common.h"
 
#define ABS_ODOM_SAVE_PERIOD (1000 * 60 * 10)

int32_t send_odom(COMPLEX_def post,COMPLEX_def speed,double pTheta,double sTheta,double absOdom);
    
     
int32_t odom_init(void);       
double  wheelTheta( WHEEL_STEER_def  *steer);    
double  wheelTheta485( WHEEL_STEER_def  *steer);
double  wheelSpeedTrans( WHEEL_STEER_def  *steer);
#ifdef __cplusplus
}
#endif


#endif











