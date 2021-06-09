/********************************************************************************

 * @file
 * @author  luys
 * @version V1.0.0
 * @date    06-20-2017
 * @brief
 *********************************************************************************/

#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_	 
#include "chassis_common.h"
#include <rtthread.h>
//////////////////////////////////////////////////////////////////////////////////	 
#ifdef __cplusplus
extern "C" {
#endif																    

/*******************************************************/

#define  DF_REMOTE_CHANNEL_CENTER               1000  /* PWM, yao gan zhongwi*/
#define DF_REMOTE_CHANNEL_RANGE                 700  /* scalling range   biao yao   */
#define DF_CHASSIS_GEAR_RANGE                  0.4f   /* you men  youmen you men +- 0.4 ms*/
#define DF_CHASSIS_GEAR_MIN_VALUE              0.6f   /*you men zhongwei*/
#define DF_CHASSIS_THROTTLE_MIN_VALUE          0.1f   /* startup  yao kong kaiji ,can't in the zui da youmen.        */

#define DF_MIN_RUN_SPEED                        0.01f  /*yaokong*/
#define DF_REMOTE_DEAT_ZONE                     40       /* joystick deadzone*/

/*shawn: what is the meanning*/
#define DF_MIN_RUN_ANGLE_SPEED                  0.01f     /* rad/s*/
//#define DF_MIN_RUN_ANGLE_SPEED                  0.01f
#define DF_RELEASE_TRAVEL_MOTOR                  1.5f     /*shi shu*/

#define DF_MIN_TURN_ON_THROTTLE                 (DF_CHASSIS_GEAR_MIN_VALUE + 0.1f)

/*no use*/
#define DF_CALIBRATION_THROTTLE                  1.0f
#define DF_CALIBRATION_VALUE                     0.9f 
#define DF_CALIBRATION_ERROR_ANGLE               0.05f 
#define DF_CALIBRATION_ENTER_ANGLE               (-2.0f * PI) 
/*no use*/

#define DF_BRAKE_ANGLE_REAL               1.0f
#define DF_RELEASE_ANGLE_REAL             2.0f
#define DF_CALIBRATION_OPEN_TIME   (0XF * 120)

extern u8 ErrState;

//�Զ���
void enterCalibration(void);
void fromMasterControl(COMPLEX_def speed, double theta);
void fromRemote(int ch1, int ch2, int ch3, int ch4, int ch7, int ch8, int flag);
int32_t controller_init(void);
int getCaliJson(char *res, char *web_name);
int getOtherJson(char *res, char *web_name);
void enterCalibration(void);
void enterCalibrationWeb(rt_bool_t is_web);
void fromControllerWebCmd(char *group, char *cmd);
#ifdef __cplusplus
}
#endif

#endif

