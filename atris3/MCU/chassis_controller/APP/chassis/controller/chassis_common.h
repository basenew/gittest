
#ifndef __CHASSIS_COMMON_H
#define __CHASSIS_COMMON_H	 

//////////////////////////////////////////////////////////////////////////////////	 
#ifdef __cplusplus
 extern "C" {
#endif																    
#include "complex.h"
#include "can2.h"
#include "app_cfg.h"
#include "priority.h"
#define DF_CENTER_I_FILTER_CONSTANT 3000
#define DF_WEICHANG_MOTOR               1
#define DF_ZHOUNHANXING_MOTOR           2
#define DF_MOTOR_PRODUCTER              DF_ZHOUNHANXING_MOTOR
#define DF_WHELL_BASE                 	0.490f //M 轴距
#define DF_TRACK_WIDTH                  0.470f //M 轮距
#define DF_THREAD_STATIC_MEMORY         1 
#define DF_TIMING_BELT_REDUCTION_RATIO                 	1.5f //同步带减速比
#if DF_MOTOR_PRODUCTER == DF_ZHOUNHANXING_MOTOR
    #define DF_PLANET_GEAR_REDUCTION_RATIO                  50.0f //行星齿轮减速比
#else
    #define DF_PLANET_GEAR_REDUCTION_RATIO                  37.14f //行星齿轮减速比     
#endif
#define DF_WHEEL_DIAMATER                 				0.250f //车轮径
#define DF_MINUTE                                       60.0f
#define DF_HELISHI_RPM_UNIT                             0.1f
#define DF_MINIMUM_STEERING_ANGLE                 		-(PI/2)//-90.0 //转向机最小角度
#define DF_MAXIMUM_STEERING_ANGLE                 		(PI/2)//90.0 //转向机最大角度

#define DF_ENCODE_485_ERROR_ANGLE                       ((PI * 5) / 180.0f)

#if DF_MOTOR_PRODUCTER == DF_ZHOUNHANXING_MOTOR
    #define DF_ENCODER_RESOLUTION                 	    	(2500 * 4) //电机编码器线数
#else
    #define DF_ENCODER_RESOLUTION                 	    	4000 //电机编码器线数  
#endif


#define DF_ENCODER485_RESOLUTION                 	    16384 //转向绝对编码器线数     
#define DF_ENCODER_ODOM                 	    	    (DF_ENCODER485_RESOLUTION * 16) //电机编码器线数

#define DF_FINE_TUNE_VALUE                               ((int)((0.1f * DF_ENCODER485_RESOLUTION) /(ANGLE_PI * 2)))
#define DF_FINE_TUNE_SAVE_TIME   (5 * 100)
#define DF_STEERING_ENPTY                 	    	0x7fffffff //电机编码器线数




#define DF_LEFT_FRONT_STEERING_ID                 	    0X21 //左前转向ID号
#define DF_LEFT_FRONT_DRIVE_ID                 	    	0X11 //左前驱动ID号
#define DF_LEFT_FRONT_STEERING_DIRECT                	-1 //左前转向机方向
#define DF_LEFT_FRONT_DRIVE_DIRECT                	    1 //左前驱动方向


#define DF_RIGHT_FRONT_STEERING_ID                 	    0X22 //右前转向ID号
#define DF_RIGHT_FRONT_DRIVE_ID                 	    0X12 //右前驱动ID号
#define DF_RIGHT_FRONT_STEERING_DIRECT                	-1 //左前转向机方向
#define DF_RIGHT_FRONT_DRIVE_DIRECT                	    -1 //左前驱动方向

#define DF_LEFT_REAR_STEERING_ID                 	    0X24 //左后转向ID号
#define DF_LEFT_REAR_DRIVE_ID                 	    	0X14 //左后驱动ID号
#define DF_LEFT_REAR_STEERING_DIRECT                	-1 //左前转向机方向
#define DF_LEFT_REAR_DRIVE_DIRECT                	    1 //左前驱动方向

#define DF_RIGHT_REAR_STEERING_ID                 	    0X23 //右后转向ID号
#define DF_RIGHT_REAR_DRIVE_ID                 	    	0X13 //右后驱动ID号
#define DF_RIGHT_REAR_STEERING_DIRECT                	-1 //左前转向机方向
#define DF_RIGHT_REAR_DRIVE_DIRECT                	    -1 //左前驱动方向

#define DF_WHEEL_STEER_NUM                 	    		4 //驱动桥臂个数

#define DF_LEFT_FRONT                 	    			0


#define DF_RIGHT_FRONT									1


#define DF_LEFT_REAR									2

#define DF_RIGHT_REAR									3

/*set to motor driver */
#define DF_INIT_ZERO_SPEED                              500
#define DF_INIT_ZERO_INC                                500
#define DF_INIT_ZERO_TORQUE                             700
#define DF_INIT_ZERO_DEAL                               400             /*deaccessary*/
#define DF_STEERING_TPDO_POSITION_TORQUE                0X180
#define DF_STEERING_TPDO_485ENCODE                      0X280
#define DF_STEERING_TPDO_ERROR_CODE_485                  0X280

#define DF_DRIVE_TPDO_ENCODE_SPEED                  0X180
#define DF_DRIVE_TPDO_ERROR_CODE_485                  0X280
#define DF_STEER_ZERO_VALUE                         "steer_zero_value"
#define DF_ABS_ODOMETER                             "absolute_odometer"


#define ABS(v)                 ((v > 0)?v:(-v))


typedef struct
{
  char name[16];
  unsigned char steeringId;
  unsigned char driveId;

  float  currentAngle;

  float  currentSpeed;
  int    currentSteeringEncodeValue;
  int    currentSteering485EncodeValue;
  int    currentDriveEncodeValue;
  int    currentSteeringtorque;
  int    currentDrivetorque;
    
  int    currentSteeringSpeed;
  int    currentDriveSpeed;
    
  int    steeringCurrent;
  int    driveCurrent;
   
  int    steeringDirect;
  int    driveDirect;
  
  int    backEncodeValue;

  int    wheelDiamater;
  
  int    minimumAngle;
  int    maximumAngle;
  int    minimumAngle485;
  int    maximumAngle485;
  int    center485;  
  int    centerI;  
  int64_t  integral_centerI;  
  int steering_error_code;
  int steering_motor_temperature;
  int steering_version_year;
  int steering_version_date;
  
  int drive_version_year;
  int drive_version_date;
  
  int drive_error_code;
  rt_bool_t abs_angle_error;
  int drive_motor_temperature;
  int drive_power_pin;
  int steering_power_pin;  
  double controlAngle;
  int controlSpeed;
  
  COMPLEX_def axlePoint;
  int init_error_code;

}WHEEL_STEER_def;



u8 Contol_Mode_SET(u8 CANopen_ID,u8 CANopen_mode);
/*激活节点*/
u8 CANopen_Activate(u8 CANopen_ID);

int CHECK_UPGRADE_STATUS(u8 CANopen_ID);
int CANopen_get_version_year(int motorId);
int CANopen_get_version_date(int motorId);
u8 SDO_Write_OD(u8 CANopen_ID,u8 CMD, u16 Index, u8 SubIndex, u32 DATA);
u8 setRestoreFactory(u8 id);
u8 setId(u8 default_ID,u8 CANopen_ID);
u8 CANopen_learn(void);
u8 CANopen_Motor_Self_learn(u8 CANopen_ID);

void setDefaultId(void);
int getDefaultId(void);

u32 SDO_Read_OD(u8 CANopen_ID,u8 CMD, u16 Index, u8 SubIndex);

	
int CANopen_PV_Init(int motorId);

int CANopen_PV_SET(int motorId,u32 Acc,u32 Dec,s32 TargetVelocity);


void Motor_PV_Zero(int motorId);

void Motor_PV_Go(int motorId);
void Motor_PV_Left(int motorId);

void Motor_PV_Right(int motorId);
int Motor_Read_Speed(int motorId);
int Motor_Read_Position_Speed(int motorId);
void set_deit(int motorId);

int Motor_PV_Back(int motorId);
//PP canopen设置
int CANopen_PP_Init(int motorId);

void CANopen_PP_Set_RUN(int motorId,s32 TargetPosition,u32 ProfileVelocity);

void Motor_PP_Trigger(int motorId);
void Motor_PP_SetEncodeValue(int motorId,int value);


//PT canopen设置
void CANopen_PT_Init(int motorId);

void CANopen_PT_Set(int motorId,s16 TargetTorque);


void Motor_enable(int motorId);

void Motor_Disable(int motorId);

void Motor_ERRtest(void);
u8 MotorOfflineSet(int motorId,int t);
u8 MotorErrorClear(int motorId);

void Motor_read(void);
void dealCanRx(CanRxMsg msg);

extern WHEEL_STEER_def  leftFront;
extern WHEEL_STEER_def  rightFront;
extern WHEEL_STEER_def  leftRear;
extern WHEEL_STEER_def  rightRear;

void wheel_steering_Init(void);
rt_bool_t calibrationSteering(void);
void initSteeringZero(void);


rt_bool_t  axleRun(WHEEL_STEER_def  *axle,COMPLEX_def speed, COMPLEX_def angS);

double getsteeringAngleF(WHEEL_STEER_def  *axle);
double getsteering485AngleF(WHEEL_STEER_def  *axle);
//#define runDirect(axle) CANopen_PV_SET( (axle)->driveId, 200,200,(axle)->controlSpeed)
//#define stopDirect(axle) CANopen_PV_SET( (axle)->driveId, 200,200,0)
#define runDirect(axle) CANopen_PV_SET( (axle)->driveId, 200,100,(axle)->controlSpeed)
#define stopDirect(axle) CANopen_PV_SET( (axle)->driveId, 200,100,0)
int getCalibrationProgress(void);
void setOneSteeringZero(WHEEL_STEER_def  *steer);
void oneSteeringZeroFilter(WHEEL_STEER_def  *steer);
void SteeringMoveSpeed(WHEEL_STEER_def  *steer,float speed);
void SteeringMoveSetp(WHEEL_STEER_def  *steer,float rad);
#ifdef __cplusplus
}
#endif


#endif











