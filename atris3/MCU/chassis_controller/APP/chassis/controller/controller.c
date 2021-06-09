/********************************************************************************

 * @file    sys.h
 * @author  luys
 * @version V3.0.0
 * @date    12-20-2017
 * @brief

 *********************************************************************************/
#include "complex.h"
#include "controller.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "e_stop_detect.h"
#include "brake.h"
#include <httpd.h>
#include "common.h"
#include "chassis_common.h"
#include "chassis_odom.h"
#include "diagnosis.h"
#include <easyflash.h>
#include "json.h"
#include "power_management.h"
#include "power_ctrl.h"
#define LOG_TAG              "control"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>
#include "motor_upgrade.h"
#include "finsh.h"

extern volatile uint8_t motor_can_init_state_;
/*brake by BA enable/disable. 1: brake by BA, 0: brake by parallel speed 0 control*/
u8 var_BA_flag;
extern uint8_t cliff_detect_flag;



static u8 ChassisClearWheelError(const WHEEL_STEER_def  *axle);

rt_bool_t chassisControl(COMPLEX_def LV, COMPLEX_def RV) {
	int runline = 0;

	static rt_bool_t motor_isOpen = RT_FALSE;
	int tem_rv_real = (int) RV.real;

	//rt_kprintf("chassisRV.real:%d imag:%d\r\n",(int)(RV.real*1000),(int)(RV.imag*1000));
	/*shawn : why it is release travel motor is 1.5?*/
	/*shawn :when disalbe motor ,why  RV.real>1.5
	 what happend, when RV.real<=1.5m/s , it should be the angle value is  then disable the travel motor:
	 travel motor:
	 why don't disable the steering motor.
	 steering motor: 1, 1.5, 100 RV.real	*/
	/* 1: ba bazi
	 2:>1.5 shi fang
	 100: web zhuan xiangji wei tiao
	 */
	if (motor_isOpen && (RV.real > DF_RELEASE_TRAVEL_MOTOR)) {
		motor_isOpen = RT_FALSE;
		if (var_BA_flag != 0) {
			Motor_Disable(0x11);
			Motor_Disable(0x12);
			Motor_Disable(0x13);
			Motor_Disable(0x14);
		}

		//Motor_Disable(0x21);
		//Motor_Disable(0x22);
		// Motor_Disable(0x23);
		//Motor_Disable(0x24);

	}
	/*shawn: once power on  enable the motor when motor is open */
	if ((!motor_isOpen) && (RV.real == 0.0f)) {
		Motor_enable(0x11);
		Motor_enable(0x12);
		Motor_enable(0x13);
		Motor_enable(0x14);
		Motor_enable(0x21);
		Motor_enable(0x22);
		Motor_enable(0x23);
		Motor_enable(0x24);
		motor_isOpen = RT_TRUE;
	}
	/*after one cycle to rebursement the angle */
	oneSteeringZeroFilter(&leftFront);
	oneSteeringZeroFilter(&rightFront);
	oneSteeringZeroFilter(&leftRear);
	oneSteeringZeroFilter(&rightRear);

	/*shawn: LV is line speed, RV is angle speed. w*/
	/*shawn: first run steering motor */
	if (axleRun(&leftFront, LV, RV))
		runline |= 1;
	if (axleRun(&rightFront, LV, RV))
		runline |= 2;
	if (axleRun(&leftRear, LV, RV))
		runline |= 4;
	if (axleRun(&rightRear, LV, RV))
		runline |= 8;

	if (!motor_isOpen)
		return RT_TRUE;

	/*then run travel motor*/
	/*shawn: if the line speed is more then 50.0 what is the unit? m/s? rpm? then runline is not f. travel motor  will be stop*/
	if (runline == 0xf) {
		runDirect(&leftFront);
		runDirect(&rightFront);
		runDirect(&leftRear);
		runDirect(&rightRear);
	} else {
		stopDirect(&leftFront);
		stopDirect(&rightFront);
		stopDirect(&leftRear);
		stopDirect(&rightRear);
	}

	return RT_TRUE;
}

void fromMasterControl(COMPLEX_def speed, double theta) {
	COMPLEX_def chassisRV;                //��ת�ٶ� rad/s
	COMPLEX_def chassisLV;                //ʸ���ٶ� m/s

	chassisLV = speed;
	chassisRV.imag = theta;
	chassisRV.real = 0.0f;

	controlChassis(chassisLV, chassisRV, MASTER_CONTROL_PRIO);
	if (0)
		LOG_I("speed  x:%d,y:%d,th:%d \r\n",
				(int)(chassisLV.real*1000),
				(int)(chassisLV.real*1000),
				(int)(chassisRV.imag * 180 / PI));
}

#define DF_SET_RESTORE_FATORY       1
#define DF_SET_SET_ID               2
#define DF_SET_SELF_LEARN_SET_ID    3

volatile rt_bool_t is_calibration = RT_FALSE;
volatile int is_set_motor_driver_id = 0;
volatile rt_bool_t is_motor_driver_upgrade = RT_NULL;
volatile rt_bool_t is_calibration_web = RT_FALSE;

void enterCalibration(void) {

	is_calibration = RT_TRUE;
}

void enterMotorUpgrade(int mode) {

	is_motor_driver_upgrade = mode;
}
void enterCalibrationWeb(rt_bool_t is_web) {

	is_calibration_web = is_web;
}
volatile int is_fine_tune_time_inc = -1;
void savecalibration(void) {
	char tem[64];
	if (is_fine_tune_time_inc >= 0)
		is_fine_tune_time_inc--;
	if (is_fine_tune_time_inc == 0) {
		rt_snprintf(tem, sizeof(tem), "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
				leftFront.maximumAngle485, leftFront.minimumAngle485,
				rightFront.maximumAngle485, rightFront.minimumAngle485,
				leftRear.maximumAngle485, leftRear.minimumAngle485,
				rightRear.maximumAngle485, rightRear.minimumAngle485);
		LOG_I("steering zero :%s\r\n",tem);
		ef_set_and_save_env(DF_STEER_ZERO_VALUE, tem);
	}
}

void fromControllerWebCmd(char *group, char *cmd) {

	if (0 == strcmp(group, "cali")) {

		if (!strcmp(cmd, "steer")) {
			enterCalibration();
		} else if (!strcmp(cmd, "lfl")) {
			leftFront.maximumAngle485 += DF_FINE_TUNE_VALUE;
			leftFront.minimumAngle485 += DF_FINE_TUNE_VALUE;
			setOneSteeringZero(&leftFront);
			is_fine_tune_time_inc = DF_FINE_TUNE_SAVE_TIME;
		} else if (!strcmp(cmd, "lfr")) {
			leftFront.maximumAngle485 -= DF_FINE_TUNE_VALUE;
			leftFront.minimumAngle485 -= DF_FINE_TUNE_VALUE;
			setOneSteeringZero(&leftFront);
			is_fine_tune_time_inc = DF_FINE_TUNE_SAVE_TIME;
		} else if (!strcmp(cmd, "rfl")) {
			rightFront.maximumAngle485 += DF_FINE_TUNE_VALUE;
			rightFront.minimumAngle485 += DF_FINE_TUNE_VALUE;
			setOneSteeringZero(&rightFront);
			is_fine_tune_time_inc = DF_FINE_TUNE_SAVE_TIME;
		} else if (!strcmp(cmd, "rfr")) {
			rightFront.maximumAngle485 -= DF_FINE_TUNE_VALUE;
			rightFront.minimumAngle485 -= DF_FINE_TUNE_VALUE;
			setOneSteeringZero(&rightFront);
			is_fine_tune_time_inc = DF_FINE_TUNE_SAVE_TIME;
		} else if (!strcmp(cmd, "ltl")) {
			leftRear.maximumAngle485 += DF_FINE_TUNE_VALUE;
			leftRear.minimumAngle485 += DF_FINE_TUNE_VALUE;
			setOneSteeringZero(&leftRear);
			is_fine_tune_time_inc = DF_FINE_TUNE_SAVE_TIME;
		} else if (!strcmp(cmd, "ltr")) {
			leftRear.maximumAngle485 -= DF_FINE_TUNE_VALUE;
			leftRear.minimumAngle485 -= DF_FINE_TUNE_VALUE;
			setOneSteeringZero(&leftRear);
			is_fine_tune_time_inc = DF_FINE_TUNE_SAVE_TIME;
		} else if (!strcmp(cmd, "rtl")) {
			rightRear.maximumAngle485 += DF_FINE_TUNE_VALUE;
			rightRear.minimumAngle485 += DF_FINE_TUNE_VALUE;
			setOneSteeringZero(&rightRear);
			is_fine_tune_time_inc = DF_FINE_TUNE_SAVE_TIME;
		} else if (!strcmp(cmd, "rtr")) {
			rightRear.maximumAngle485 -= DF_FINE_TUNE_VALUE;
			rightRear.minimumAngle485 -= DF_FINE_TUNE_VALUE;
			setOneSteeringZero(&rightRear);
			is_fine_tune_time_inc = DF_FINE_TUNE_SAVE_TIME;
		} else if (!strcmp(cmd, "id_set")) {
			LOG_I("from web cmd set MOTOR DRIVER ID");
			is_set_motor_driver_id = DF_SET_SET_ID;
		} else if (!strcmp(cmd, "self_learn_id_set")) {
			LOG_I("from web cmd set MOTOR DRIVER ID");
			is_set_motor_driver_id = DF_SET_SELF_LEARN_SET_ID;
		} else if (!strcmp(cmd, "refactory")) {
			LOG_I("from web cmd set MOTOR DRIVER ID");
			is_set_motor_driver_id = DF_SET_RESTORE_FATORY;
		}

	} else if (0 == strcmp(group, "other")) {
		if (0 == strcmp(cmd, "BA_flag=on")) {
			var_BA_flag = 1;
		} else if (0 == strcmp(cmd, "BA_flag=off")) {
			var_BA_flag = 0;
		} else if (0 == strcmp(cmd, "cliff_detect_flag=on")) {
			cliff_detect_flag = 1;
		} else if (0 == strcmp(cmd, "cliff_detect_flag=off")) {
			cliff_detect_flag = 0;
		}
	}

}
#define DF_OFF_DELAY_TIME 2000
#define DF_ON_DELAY_TIME 5000
#define DF_STEER_TYPE "steer"
#define DF_DRIVER_TYPE "driver"
char *web_json_str_p;
char motor_id_info[64];
rt_bool_t singleMotorDriverRestoreFactory(WHEEL_STEER_def *axle,
		char *motorType, int power_switch) {
	int id;
	int i;
	int default_id;
	if (!strcmp(motorType, DF_STEER_TYPE))
		id = axle->steeringId;
	else
		id = axle->driveId;
	for (i = 0; i < 20; i++) {
		rt_thread_mdelay(DF_OFF_DELAY_TIME);
		if (!web_json_str_p)
			break;
	}
	sprintf(motor_id_info, "%s-%s Restore Factory(%d)", axle->name, motorType,
			id);
	LOG_I("%s",motor_id_info);
	web_json_str_p = motor_id_info;
	for (i = 0; i < 20; i++) {
		rt_thread_mdelay(DF_OFF_DELAY_TIME);
		if (!web_json_str_p)
			break;
	}
	sprintf(motor_id_info, "...start and power off");
	LOG_I("%s",motor_id_info);
	web_json_str_p = motor_id_info;

	motor_power_off();
	rt_thread_mdelay(DF_OFF_DELAY_TIME);

	for (i = 0; i < 20; i++) {
		rt_thread_mdelay(DF_OFF_DELAY_TIME);
		if (!web_json_str_p)
			break;
	}
	sprintf(motor_id_info, "...re power");
	LOG_I("%s",motor_id_info);
	web_json_str_p = motor_id_info;
	setDefaultId();
	power_set_single_power(power_switch, POWER_ON);
	rt_thread_mdelay(DF_ON_DELAY_TIME);
	default_id = getDefaultId();
	sprintf(motor_id_info, "...restore factory start");
	LOG_I("%s",motor_id_info);
	rt_thread_mdelay(1000);
	if (setRestoreFactory(default_id) == 1) {
		sprintf(motor_id_info, "...restore factory false");
		LOG_I("%s",motor_id_info);
		rt_thread_mdelay(1000);
		return RT_FALSE;
	}
	sprintf(motor_id_info, "...restore factory OK");
	LOG_I("%s",motor_id_info);
	rt_thread_mdelay(1000);
	return RT_FALSE;
}

rt_bool_t setSingleMotorDriverId(WHEEL_STEER_def *axle, char *motorType,
		int power_switch) {
	int id;
	int i;
	int default_id;
	if (!strcmp(motorType, DF_STEER_TYPE))
		id = axle->steeringId;
	else
		id = axle->driveId;
	for (i = 0; i < 20; i++) {
		rt_thread_mdelay(DF_OFF_DELAY_TIME);
		if (!web_json_str_p)
			break;
	}
	sprintf(motor_id_info, "%s-%s set id(%d)", axle->name, motorType, id);
	LOG_I("%s",motor_id_info);
	web_json_str_p = motor_id_info;
	for (i = 0; i < 20; i++) {
		rt_thread_mdelay(DF_OFF_DELAY_TIME);
		if (!web_json_str_p)
			break;
	}
	sprintf(motor_id_info, "...start and power off");
	LOG_I("%s",motor_id_info);
	web_json_str_p = motor_id_info;

	motor_power_off();
	rt_thread_mdelay(DF_OFF_DELAY_TIME);

	for (i = 0; i < 20; i++) {
		rt_thread_mdelay(DF_OFF_DELAY_TIME);
		if (!web_json_str_p)
			break;
	}
	sprintf(motor_id_info, "...re power");
	LOG_I("%s",motor_id_info);
	web_json_str_p = motor_id_info;
	setDefaultId();
	power_set_single_power(power_switch, POWER_ON);
	rt_thread_mdelay(DF_ON_DELAY_TIME);
	default_id = getDefaultId();
	if (is_set_motor_driver_id == DF_SET_SELF_LEARN_SET_ID) {
		sprintf(motor_id_info, "...start self learn");
		LOG_I("%s",motor_id_info);
		web_json_str_p = motor_id_info;
		CANopen_learn();
		CANopen_Motor_Self_learn(default_id);
		rt_thread_mdelay(DF_ON_DELAY_TIME);
		rt_thread_mdelay(DF_ON_DELAY_TIME);
		sprintf(motor_id_info, "...self learn OK");
	}LOG_I("%s",motor_id_info);
	web_json_str_p = motor_id_info;
	rt_thread_mdelay(DF_ON_DELAY_TIME);
	if (default_id == id) {
		for (i = 0; i < 20; i++) {
			rt_thread_mdelay(DF_OFF_DELAY_TIME);
			if (!web_json_str_p)
				break;
		}
		sprintf(motor_id_info, "...id is OK,not set");
		web_json_str_p = motor_id_info;
		LOG_I("%s",motor_id_info);
		return RT_TRUE;
	}
	if (setId(default_id, id) == 0) {

		for (i = 0; i < 20; i++) {
			rt_thread_mdelay(DF_OFF_DELAY_TIME);
			if (!web_json_str_p)
				break;
		}
		sprintf(motor_id_info, "...id is set OK");
		web_json_str_p = motor_id_info;
		LOG_I("%s",motor_id_info);
		return RT_TRUE;
	} else {
		for (i = 0; i < 20; i++) {
			rt_thread_mdelay(DF_OFF_DELAY_TIME);
			if (!web_json_str_p)
				break;
		}
		sprintf(motor_id_info, "...id is set FALSE");
		web_json_str_p = motor_id_info;
		LOG_I("%s",motor_id_info);

	}

	return RT_FALSE;
}

void setMotorDriverId(void) {
	rt_bool_t error;
	int i;
	web_json_str_p = RT_NULL;
	error = setSingleMotorDriverId(&leftFront, DF_STEER_TYPE,
	POWER_24V_LEFT_FRONT_STEER);
	error &= setSingleMotorDriverId(&leftFront, DF_DRIVER_TYPE,
	POWER_24V_LEFT_FRONT_DIRECT);

	error &= setSingleMotorDriverId(&rightFront, DF_STEER_TYPE,
	POWER_24V_RIGHT_FRONT_STEER);
	error &= setSingleMotorDriverId(&rightFront, DF_DRIVER_TYPE,
	POWER_24V_RIGHT_FRONT_DIRECT);

	error &= setSingleMotorDriverId(&leftRear, DF_STEER_TYPE,
	POWER_24V_LEFT_TEAR_STEER);
	error &= setSingleMotorDriverId(&leftRear, DF_DRIVER_TYPE,
	POWER_24V_LEFT_TEAR_DIRECT);

	error &= setSingleMotorDriverId(&rightRear, DF_STEER_TYPE,
	POWER_24V_RIGHT_TEAR_STEER);
	error &= setSingleMotorDriverId(&rightRear, DF_DRIVER_TYPE,
	POWER_24V_RIGHT_TEAR_DIRECT);
	for (i = 0; i < 20; i++) {
		rt_thread_mdelay(DF_OFF_DELAY_TIME);
		if (!web_json_str_p)
			break;
	}
	sprintf(motor_id_info, "WEB set id is %s", error ? "OK ^_^" : "FALSE!");
	web_json_str_p = motor_id_info;
	LOG_I("%s",motor_id_info);

}

void motorDriverRestoreFactory(void) {
	rt_bool_t error;
	int i;
	web_json_str_p = RT_NULL;
	error = singleMotorDriverRestoreFactory(&leftFront, DF_STEER_TYPE,
	POWER_24V_LEFT_FRONT_STEER);
	error &= singleMotorDriverRestoreFactory(&leftFront, DF_DRIVER_TYPE,
	POWER_24V_LEFT_FRONT_DIRECT);

	error &= singleMotorDriverRestoreFactory(&rightFront, DF_STEER_TYPE,
	POWER_24V_RIGHT_FRONT_STEER);
	error &= singleMotorDriverRestoreFactory(&rightFront, DF_DRIVER_TYPE,
	POWER_24V_RIGHT_FRONT_DIRECT);

	error &= singleMotorDriverRestoreFactory(&leftRear, DF_STEER_TYPE,
	POWER_24V_LEFT_TEAR_STEER);
	error &= singleMotorDriverRestoreFactory(&leftRear, DF_DRIVER_TYPE,
	POWER_24V_LEFT_TEAR_DIRECT);

	error &= singleMotorDriverRestoreFactory(&rightRear, DF_STEER_TYPE,
	POWER_24V_RIGHT_TEAR_STEER);
	error &= singleMotorDriverRestoreFactory(&rightRear, DF_DRIVER_TYPE,
	POWER_24V_RIGHT_TEAR_DIRECT);
	for (i = 0; i < 20; i++) {
		rt_thread_mdelay(DF_OFF_DELAY_TIME);
		if (!web_json_str_p)
			break;
	}
	sprintf(motor_id_info, "WEB restory factory is %s",
			error ? "OK ^_^" : "FALSE!");
	web_json_str_p = motor_id_info;
	LOG_I("%s",motor_id_info);

}

static void controller_thread_entry(void *_param) {

	while (power_motor_status_get() != MOTOR_WORK)
		rt_thread_mdelay(1000);
	

	while (1) {
		if ((is_set_motor_driver_id == DF_SET_SET_ID)
				|| (is_set_motor_driver_id == DF_SET_SELF_LEARN_SET_ID)) {
			setMotorDriverId();
			motor_power_off();
			rt_thread_mdelay(DF_OFF_DELAY_TIME);
			is_set_motor_driver_id = RT_FALSE;
		}
		/*shawn: what is the meaning DF_SET_RESTORE_FATORY */
		if (is_set_motor_driver_id == DF_SET_RESTORE_FATORY) {
			motorDriverRestoreFactory();
			motor_power_off();
			rt_thread_mdelay(DF_OFF_DELAY_TIME);
			is_set_motor_driver_id = RT_FALSE;
		}
		if (is_motor_driver_upgrade) {
			LOG_I("is_motor_driver_upgrade control\r\n");
			motor_power_off();
			rt_thread_mdelay(DF_ON_DELAY_TIME);
			motorUpgrade(is_motor_driver_upgrade);
			motor_power_off();
			rt_thread_mdelay(DF_ON_DELAY_TIME);
			motor_power_on();
			is_motor_driver_upgrade = RT_NULL;
		}
		while (power_motor_status_get() == MOTOR_STANDBY)
			rt_thread_mdelay(1000);
		motor_power_on();
		int e_stop_inc = 0;
		rt_bool_t first_run = true;
		COMPLEX_def RV;
		COMPLEX_def LV;
		rt_thread_mdelay(1000 * 5);
		wheel_steering_Init();
		motor_can_init_state_ = 1;
		LOG_I("remote control\r\n");

		Motor_Disable(0x11);
		Motor_Disable(0x12);
		Motor_Disable(0x13);
		Motor_Disable(0x14);

		if (get_brake_status() != BRAKE_STATUS_LOCK) {

			ChassisBreak();
		}

		while (((power_motor_status_get() == MOTOR_WORK) || (e_stop_inc < 500))
				&& (is_set_motor_driver_id == 0)
				&& (is_motor_driver_upgrade == RT_NULL)) {

			if (diagnosisControl()) {
				rt_thread_mdelay(100);
				continue;
			}

			if (is_calibration) {
				LOG_I("enter steering zero init\r\n");
				calibrationSteering();
				LOG_I("steering zero init OK\r\n");
				is_calibration = RT_FALSE;

			}

			getChassisSpeed(&LV, &RV);
			/*Shawn:get the status from upper layer, if it is break, set the RV.real =1.0f then ba zi 2: shifang. 100 ceshi mode gongchang zhuanxiangji weitiao. if the uplayer set the RV is not 1.0f then motor to zero to run*/
			if (((get_brake_status() == BRAKE_STATUS_LOCK)
					|| (power_motor_status_get() == MOTOR_STANDBY))
					&& (DF_MOTOR_PRODUCTER == DF_ZHOUNHANXING_MOTOR)
					&& (!first_run)) {
				//rt_kprintf("e_stop down\r\n");
				LV.real = 0.0f;
				LV.imag = 0.0f;
				RV.imag = 0.0f;
				/*shawn: ba zi*/

				if (var_BA_flag != 0)
					RV.real = 1.0f;
				else
					RV.real = 0.0f;

			} else if (first_run) {
				if (get_brake_status() != BRAKE_STATUS_LOCK) {
					first_run = false;
					ChassisBreak();
				
				}
						 MotorOfflineSet(0x11,300);
	           MotorOfflineSet(0x12,300);
	           MotorOfflineSet(0x13,300);
	           MotorOfflineSet(0x14,300);

			} else {
				// rt_kprintf("e_stop up\r\n");
				first_run = false;

				e_stop_inc = 0;
			}
			if (e_stop_inc < 500)
				e_stop_inc++;
			else {
				/*shawn: what is the transport mode, why it is 100 and 2.0*/
				/*what is transport mode bubai ba zi  100 :bai zhi*/
				if (chassis_transport_mode_get() == DF_ENABLE)
					LV.real = 100.0f;
				else
					RV.real = 2.0f;

			}
			/*shawn: save the zero center point abs485 to the flash*/
			if ((is_calibration_web) && (!first_run)) {
				if (fabs(LV.real) < 0.1f)
					LV.real = 100.0f;
				LV.imag = 0.0f;
				RV.imag = 0.0f;
				RV.real = 0.0f;
				e_stop_inc = 0;
				savecalibration();
			}
			if(ChassisClearWheelError(&leftFront)==0||
        ChassisClearWheelError(&rightFront)==0||
        ChassisClearWheelError(&leftRear)==0||
        ChassisClearWheelError(&rightRear)==0)
			{ 
				LV.real = 0.0f;  
				LV.imag = 0.0f;
				RV.imag = 0.0f;
				RV.real = 0.0f;
			
			}
				
			
			
			
			
			chassisControl(LV, RV);
			rt_thread_mdelay(10);
		}
		motor_power_off();
	}

}

int getCaliJson(char *res, char *web_name) {
	int len = 0;
	if (is_set_motor_driver_id) {
		if (web_json_str_p) {
			len = initJson(res, "setId");
			len = addStrToJson(res, "set_id_info", web_json_str_p);
			web_json_str_p = RT_NULL;
		} else
			len = initJson(res, "non");

	} else {
		len = initJson(res, web_name);
		len = addIntToJson(res, "calibration", getCalibrationProgress());

		len = addFoatToJson(res, "lfA",
				wheelTheta(&leftFront) * 180 * 100 / PI);
		len = addFoatToJson(res, "rfA",
				wheelTheta(&rightFront) * 180 * 100 / PI);
		len = addFoatToJson(res, "ltA", wheelTheta(&leftRear) * 180 * 100 / PI);
		len = addFoatToJson(res, "rtA",
				wheelTheta(&rightRear) * 180 * 100 / PI);

		len = addFoatToJson(res, "lf4",
				wheelTheta485(&leftFront) * 180 * 100 / PI);
		len = addFoatToJson(res, "rf4",
				wheelTheta485(&rightFront) * 180 * 100 / PI);
		len = addFoatToJson(res, "lt4",
				wheelTheta485(&leftRear) * 180 * 100 / PI);
		len = addFoatToJson(res, "rt4",
				wheelTheta485(&rightRear) * 180 * 100 / PI);

		//printf("%s(%d):%s",__FUNCTION__,len,res);
	}
	return len;
}

int getOtherJson(char *res, char *web_name) {
	int len = initJson(res, web_name);
	len = addIntToJson(res, "BA_flag", var_BA_flag);
	len = addIntToJson(res, "cliff_detect_flag", cliff_detect_flag);
	return len;
}

int32_t controller_init(void) {
	var_BA_flag = 0;/*brake by BA*/


	
	if (DF_THREAD_STATIC_MEMORY == 0) {
		rt_thread_t thread = rt_thread_create("controller_thread",
				controller_thread_entry, RT_NULL, TASK_STACK_SIZE_CONTROLLER,
				TASK_PRIORITY_CONTROLLER, 10);

		if (thread != RT_NULL) {
			rt_thread_startup(thread);
		} else {
			return -1;
		}
	} else {
		static struct rt_thread controller_thread;
		ALIGN(RT_ALIGN_SIZE)
		static char controller_thread_stack[TASK_STACK_SIZE_CONTROLLER];
		rt_err_t result = RT_EOK;
		result = rt_thread_init(&controller_thread, "controller_thread",
				controller_thread_entry, RT_NULL, &controller_thread_stack[0],
				sizeof(controller_thread_stack),
				TASK_PRIORITY_CONTROLLER, 10);

		if (result == RT_EOK)
			rt_thread_startup(&controller_thread);
		else
			LOG_I("%s thread create failed.",__FUNCTION__);

	}

	return 0;
}

/******************************************************************************************************************/
static void Var_EnalbeDisable(uint8_t argc, char **argv) {
	if (argc < 2 || argc > 3) {
		rt_kprintf("Please input: Var Enable Disable <1/0> 1:enable 0:disable\n  var_BA_flag  \n");
	} else {
		if ((!strncmp("var_BA", argv[1], 6))) {
			var_BA_flag = atoi(argv[2]);
		}

	}
}
MSH_CMD_EXPORT(Var_EnalbeDisable, Variable Enable Disable);




static u8 ChassisClearWheelError(const WHEEL_STEER_def  *axle)
{
  u8 ret =1;
	if(axle->steering_error_code !=0)
	{
		MotorErrorClear(axle->steeringId);
		LOG_E("motor error  %s    , %d   ,   %d \n", axle->name,   axle->steeringId,  axle->steering_error_code);

		ret =0;
	}
	else if(axle->drive_error_code !=0)
	{
		
		MotorErrorClear(axle->driveId);
		LOG_E("motor error  %s    , %d   ,   %d \n", axle->name,   axle->driveId,   axle->drive_error_code   );
		ret =0;
	}
	else
	{
	}
		
  return ret; 
}
