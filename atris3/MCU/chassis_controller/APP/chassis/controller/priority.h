#ifndef __CMD_VEL_PRIORITY_H
#define __CMD_VEL_PRIORITY_H	 

//////////////////////////////////////////////////////////////////////////////////	 
#ifdef __cplusplus
 extern "C" {
#endif	
#include "can2.h"     
#include "complex.h"
 #include "chassis_common.h"
#include "controller.h"
#include <rtthread.h>   

#define DF_PROIRITY_TOTAL  0XFFFFFFFF
#define DF_PROIRITY_MAX    0XF0000000
#define DF_PROIRITY_DEC    0X11111111
#define DF_PROIRITY_BIT_WIDTH  4
#define DF_PROIRITY_MIN   0XF
	 /*shawn: what is the meaning?*/
#define DF_PROIRITY_BRAKE_FULL   (0XF * 5)  /*you kongzhi sudu zhi ,xie zhe ge zhi  you shu ju lai liao ,jiu dijian ,jian dao 0 sha che */
#define DF_PROIRITY_BRAKE   (0XF * 2)       /*jian dao 2 bai ba zi  */
#define DF_PROIRITY_RELEASE   (0)           /*shi fang xingzou (travel) dianji* release motor*/
	 #define DF_CALIBRATION_OPEN_TIME   (0XF * 120) /*what is the unit 120 second   tick: 16.7ms ?*/
     
enum PROIRITY{
MASTER_CONTROL_PRIO     = 0xf,    
REMOTE_PRIO             =0XF0,
AUTO_RUN_PRIO           =0XF00,  /*power on then auto run set to ba*/       
};     
     
rt_bool_t getCalibrationSwitchStatus(void);  
rt_bool_t controlChassis(COMPLEX_def LV,COMPLEX_def RV,int prio);

rt_bool_t getChassisSpeed(COMPLEX_def *LV,COMPLEX_def *RV);
int32_t priority_init(void);    
rt_bool_t ChassisBreak(void);
#ifdef __cplusplus
}
#endif


#endif
