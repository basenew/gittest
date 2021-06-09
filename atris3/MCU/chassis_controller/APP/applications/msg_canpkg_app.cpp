/*
 * File      : tinyros_entries.cpp
 * This file is part of tinyros
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-22     Pinkie.Fu    initial version
 * 2020-07-22     xiaofeng.wu  modify for msg canpkg
 */

#include <rtthread.h>
#include <sys/time.h>
#include "common.h"
#include "app_cfg.h"
#include "msg_canpkg.h"
#include "msg_canpkg_app.h"
#include "remote_ctrl.h"
#include "power_management.h"
#include "ntc_sample.h"
#include "e_stop_detect.h"
#include "bms.h"
#include "power_ctrl.h"
#include "voltage_detect.h"
#include "sensor_detec.h"
#include "brake.h"
#include "version.h"
#include "rt_fota.h"
#include "tiny_ros/ros/time.h"
#include "chassis_common.h"
#include "chassis_odom.h"
#include "curr_sample.h"

#define LOG_TAG              "canpkg_app"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>

uint8_t cliff_detect_flag = 0;

#define  __ADD_BMS_WARN_TEST    1
#define __ADD_CHASS_ABNOR_TEST  1

#if __ADD_BMS_WARN_TEST

static uint8_t __bms_warn_data[8] = { 0xFF, };

static void bms_warn(int argc, char **argv) {
    if (2 == argc && 0 == strcmp(argv[1], "reset")) {
        __bms_warn_data[0] = 0xFF;
        rt_kprintf("OK\n") ;
        return;
    }

    if (9 == argc) {
        uint8_t a1[8];
        for (int i = 0; i < 8; ++i) {
            a1[i] = atoi(argv[1 + i]);
        }

        rt_memcpy(__bms_warn_data, a1, 8);

        rt_kprintf("OK. BMS warn data: %d %d %d %d %d %d %d %d\n", a1[0], a1[1], a1[2], a1[3], a1[4], a1[5], a1[6], a1[7]) ;

        return;
    }

USAGE:
    rt_kprintf("Usage: \n\tbms_warn reset\n\tbms_warn byte1 byte2 ... byte8\n") ;
}

MSH_CMD_EXPORT(bms_warn, bms_warn);

#endif // __ADD_BMS_WARN_TEST

#if __ADD_CHASS_ABNOR_TEST

static struct {
    int axle_index;
    tinyros::atris_msgs::chassis_abnor_info axle_data;
} __chass_abnor_data = { 0xFF, };

#define WHOLE_PART(n)       ((int)(n))
#define FRACTION_PART(n, s)    ((int)((n) * (s)) % (s))

static void chass_abnor(int argc, char *argv[]) {

    // chass_abnor reset -----

    if (2 == argc && 0 == strcmp(argv[1], "reset")) {
        __chass_abnor_data.axle_index = 0xFF;
        rt_kprintf("OK\n") ;
        return;
    }

    // chass_abnor list ----

    if (2 == argc && 0 == strcmp(argv[1], "list")) {
        const tinyros::atris_msgs::chassis_abnor_info *data = &__chass_abnor_data.axle_data;

        rt_kprintf("OK\n") ;

        if (0xFF == __chass_abnor_data.axle_index) {
            rt_kprintf("axle = ----\n") ;
        } else {
            rt_kprintf("axle = %s\n", data->axle) ;
        }

        rt_kprintf("chassis_abnor = %d\n", data->chassis_abnor) ;
        rt_kprintf("steer_error_code = %d\n", data->steer_error_code) ;
        rt_kprintf("direct_error_code = %d\n", data->direct_error_code) ;

        //        rt_kprintf("steer_485_angle = %f\n", data->steer_485_angle) ;
        //        rt_kprintf("steer_angle = %f\n", data->steer_angle) ;
        //        rt_kprintf("direct_speed = %f\n", data->direct_speed) ;
        rt_kprintf("steer_485_angle = %d.%06d\n", WHOLE_PART( data->steer_485_angle) , FRACTION_PART(data->steer_485_angle, 1000000)) ;
        rt_kprintf("steer_angle = %d.%06d\n", WHOLE_PART( data->steer_angle) ,FRACTION_PART (data->steer_angle, 1000000) ) ;
        rt_kprintf("direct_speed = %d.%06d\n", WHOLE_PART(data->direct_speed ), FRACTION_PART(data->direct_speed, 1000000) ) ;

        rt_kprintf("steer_temperature = %d\n", data->steer_temperature) ;
        rt_kprintf("direct_temperature = %d\n", data->direct_temperature) ;
        rt_kprintf("steer_torque = %d\n", data->steer_torque) ;
        rt_kprintf("direct_torque = %d\n", data->direct_torque) ;

        return;
    }

    // chass_abnor set <name> <value> -----

    if (4 == argc && 0 == strcmp(argv[1], "set")) {
        tinyros::atris_msgs::chassis_abnor_info *data = &__chass_abnor_data.axle_data;

        if (0 == strcmp(argv[2], "axle")) {
            if (0 == strcmp(argv[3], "leftFront") || 0 == strcmp(argv[3], "0")) {
                __chass_abnor_data.axle_index = 0;
                strcpy(data->axle, "leftFront");
            } else if (0 == strcmp(argv[3], "rightFront") || 0 == strcmp(argv[3], "1")) {
                __chass_abnor_data.axle_index = 1;
                strcpy(data->axle, "rightFront");
            } else if (0 == strcmp(argv[3], "leftRear") || 0 == strcmp(argv[3], "2")) {
                __chass_abnor_data.axle_index = 2;
                strcpy(data->axle, "leftRear");
            } else if (0 == strcmp(argv[3], "rightRear") || 0 == strcmp(argv[3], "3")) {
                __chass_abnor_data.axle_index = 3;
                strcpy(data->axle, "rightRear");
            } else {
                rt_kprintf("Error\n");
                goto USAGE;
            }

            rt_kprintf("OK\n") ;
            rt_kprintf("axle = %s\n", data->axle) ;

            return;
        }

        if (0 == strcmp(argv[2], "chassis_abnor")) {
            int n1 = atoi(argv[3]);
            if (0 == n1 || 1 == n1 || 2 == n1) {
                data->chassis_abnor = n1;
            } else {
                rt_kprintf("Error\n");
                goto USAGE;
            }

            rt_kprintf("OK\n") ;
            rt_kprintf("chassis_abnor = %d\n", data->chassis_abnor) ;

            return;
        }

        if (0 == strcmp(argv[2], "steer_error_code")) {
            int n1 = atoi(argv[3]);
            data->steer_error_code = (int8_t) n1;

            rt_kprintf("OK\n") ;
            rt_kprintf("steer_error_code = %d\n", data->steer_error_code) ;

            return;
        }

        if (0 == strcmp(argv[2], "direct_error_code")) {
            int n1 = atoi(argv[3]);
            data->direct_error_code = (int8_t) n1;

            rt_kprintf("OK\n") ;
            rt_kprintf("direct_error_code = %d\n", data->direct_error_code) ;

            return;
        }

        if (0 == strcmp(argv[2], "steer_485_angle")) {
            double n1 = atof(argv[3]);
            data->steer_485_angle = (float) n1;

            rt_kprintf("OK\n") ;
            rt_kprintf("steer_485_angle = %d.%06d\n", WHOLE_PART( data->steer_485_angle ), FRACTION_PART( data->steer_485_angle, 1000000 )) ;

            return;
        }

        if (0 == strcmp(argv[2], "steer_angle")) {
            double n1 = atof(argv[3]);
            data->steer_angle = (float) n1;

            rt_kprintf("OK\n") ;
            rt_kprintf("steer_angle = %d.%06d\n", WHOLE_PART( data->steer_angle), FRACTION_PART( data->steer_angle, 1000000 )) ;

            return;
        }

        if (0 == strcmp(argv[2], "direct_speed")) {
            double n1 = atof(argv[3]);
            data->direct_speed = (float) n1;

            rt_kprintf("OK\n") ;
            rt_kprintf("direct_speed = %d.%06d\n", WHOLE_PART( data->direct_speed ), FRACTION_PART( data->direct_speed, 1000000 )) ;

            return;
        }

        if (0 == strcmp(argv[2], "steer_temperature")) {
            int n1 = atoi(argv[3]);
            data->steer_temperature = (int8_t) n1;

            rt_kprintf("OK\n") ;
            rt_kprintf("steer_temperature = %d\n", data->steer_temperature) ;

            return;
        }

        if (0 == strcmp(argv[2], "direct_temperature")) {
            int n1 = atoi(argv[3]);
            data->direct_temperature = (int8_t) n1;

            rt_kprintf("OK\n") ;
            rt_kprintf("direct_temperature = %d\n", data->direct_temperature) ;

            return;
        }

        if (0 == strcmp(argv[2], "steer_torque")) {
            int n1 = atoi(argv[3]);
            data->steer_torque = (int16_t) n1;

            rt_kprintf("OK\n") ;
            rt_kprintf("steer_torque = %d\n", data->steer_torque) ;

            return;
        }

        if (0 == strcmp(argv[2], "direct_torque")) {
            int n1 = atoi(argv[3]);
            data->direct_torque = (int16_t) n1;

            rt_kprintf("OK\n") ;
            rt_kprintf("direct_torque = %d\n", data->direct_torque) ;

            return;
        }

        rt_kprintf("Error.\n") ;
        goto USAGE;
    } // if

USAGE:
    rt_kprintf("Usage:\n\tchass_abnor reset\n");
    rt_kprintf("\tchass_abnor list\n") ;
    rt_kprintf("\tchass_abnor set <name> <value>\n") ;
}

MSH_CMD_EXPORT(chass_abnor, chass_abnor);

#endif // __ADD_CHASS_ABNOR_TEST

static void rtc_sync_to(void) {
#ifdef RT_USING_RTC
    tinyros::Time time = tinyros::Time::dds();
    double sec = time.toSec();

    //struct tm cur_tm;
    
    static struct tm *tm, tm_tmp;
    
    time_t now = (time_t)sec;

    rt_kprintf("111--now: %d \n", now);
    
    
    if (now)
    {
        now += 8 * 3600;
        
        rt_kprintf("222--now: %d \n", now);
        
        //rt_enter_critical();
        //cur_tm = *(localtime(&cur_time));
        //localtime_r(&cur_time, &cur_tm);
        //rt_exit_critical();
        
        tm = gmtime_r(&now, &tm_tmp);
        
        rt_kprintf("333--rtc_sync_to: %d-%d-%d %d:%d:%d\n", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, \
                                                  tm->tm_hour, tm->tm_min, tm->tm_sec);
        rtc_sync_time(tm);

        LOG_W("rtc_sync_to: %d-%d-%d %d:%d:%d\n", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, \
                                                  tm->tm_hour, tm->tm_min, tm->tm_sec);
    }
#endif
}

static void rtc_sync(void) {
#ifdef RT_USING_RTC
    static uint32_t s_timer = 0;

    if (s_timer == 0) 
    {
        if(os_gettime_ms() - s_timer >= 2*60*1000)
        {
            s_timer = os_gettime_ms();
            rtc_sync_to();
        } else if (os_gettime_ms() < s_timer) {
            s_timer = os_gettime_ms();
        }
    }
    else
    {
        if(os_gettime_ms() - s_timer >= 10*60*1000) 
        {
            s_timer = os_gettime_ms();
            rtc_sync_to();
        } else if (os_gettime_ms() < s_timer) {
            s_timer = os_gettime_ms();
        }
    }
#endif
}

/*send battery data*/
static int32_t bat_data_post(void) {
    tinyros::atris_msgs::CanPkg msg;
    bms_data_t *pdata = RT_NULL;

    pdata = bat_get_bms_data_ptr();

    msg.cmd = CANPKG_CMD_BATTERY_INFO_REPORT;
    msg.data_i[0] = pdata->TotalVolt;
    msg.data_i[1] = pdata->TotalCurr;
    msg.data_i[2] = pdata->SOC;
    msg.data_i[3] = pdata->SOH;
    msg.data_i[4] = pdata->PackVolt;
    msg.data_i[5] = pdata->PackCap;
    msg.data_i[6] = pdata->CellVolt_High;
    msg.data_i[7] = pdata->CellVolt_Low;
    msg.data_i[8] = pdata->PackNo_CellVoltHigh;
    msg.data_i[9] = pdata->PackNo_CellVoltLow;
    msg.data_i[10] = pdata->BunchNo_CellVoltHigh;
    msg.data_i[11] = pdata->BunchNo_CellVoltLow;
    msg.data_i[12] = pdata->CellTempHigh;
    msg.data_i[13] = pdata->CellTempLow;
    msg.data_i[14] = pdata->PackNo_CellTempHigh;
    msg.data_i[15] = pdata->PackNo_CellTempLow;
    msg.data_i[16] = pdata->BunchNo_CellTempHigh;
    msg.data_i[17] = pdata->BunchNo_CellTempLow;
    msg.data_i[18] = pdata->BmsLife;
    msg.data_i[19] = pdata->ChgMosStatus;
    msg.data_i[20] = pdata->DisChgMosStatus;
    msg.data_i[21] = pdata->ExIoStatus;
    msg.data_i[22] = pdata->ChgDisChg_Cnts;
    msg.data_i[23] = pdata->SleepFlag;
    msg.data_i[24] = pdata->ChgStatus;

#if __ADD_BMS_WARN_TEST
    if (0xFF != __bms_warn_data[0]) {
        msg.data_i[25] = __bms_warn_data[0];
        msg.data_i[26] = __bms_warn_data[1];
        msg.data_i[27] = (__bms_warn_data[2] | (__bms_warn_data[3] << 8));
        msg.data_i[28] = (__bms_warn_data[4] | (__bms_warn_data[5] << 8));
        msg.data_i[29] = (__bms_warn_data[6] | (__bms_warn_data[7] << 8));
    } else {
        msg.data_i[25] = pdata->uWarn.sWarn.WarnStatus;
        msg.data_i[26] = pdata->uWarn.sWarn.WarnLevel_H;
        msg.data_i[27] = (pdata->uWarn.Warn[2] | (pdata->uWarn.Warn[3] << 8));
        msg.data_i[28] = (pdata->uWarn.Warn[4] | (pdata->uWarn.Warn[5] << 8));
        msg.data_i[29] = (pdata->uWarn.Warn[6] | (pdata->uWarn.Warn[7] << 8));
    }
#else
    msg.data_i[25] = pdata->uWarn.sWarn.WarnStatus;
    msg.data_i[26] = pdata->uWarn.sWarn.WarnLevel_H;
    msg.data_i[27] = (pdata->uWarn.Warn[2] | (pdata->uWarn.Warn[3] << 8));
    msg.data_i[28] = (pdata->uWarn.Warn[4] | (pdata->uWarn.Warn[5] << 8));
    msg.data_i[29] = (pdata->uWarn.Warn[6] | (pdata->uWarn.Warn[7] << 8));

#endif  // __ADD_BMS_WARN_TEST

    return canpkg_publish_msg(msg);
}

/*send power_down data*/
static void power_down_request_post(void) {
    uint8_t temp = 0;
    tinyros::atris_msgs::CanPkg msg;

    msg.cmd = CANPKG_CMD_POWER_ONOFF_REQUEST;

    temp = power_down_get_status_report();
    if (temp) {
        msg.data_i[0] = temp;
        canpkg_publish_msg(msg);
//		LOG_W("send power_onoff status to host: %d\n", temp);
    }
}

/*send power_mode data to sys_ctrl board*/
static void light_mode_request_post(void) {
    uint8_t temp = 0;
    tinyros::atris_msgs::CanPkg msg;

    msg.cmd = CANPKG_CMD_LIGHT_MODE_REQUEST;

    msg.data_i[0] = light_mode_get();
    msg.data_i[1] = power_mode_get();
    canpkg_publish_msg(msg);
//	LOG_W("send light_mode: %d, %d\n", msg.data_i[0], msg.data_i[1]);

}

static void chassis_diagnosis_post(void) {

    WHEEL_STEER_def *axle_p;
    static uint8_t axle_inc = 0;

#if __ADD_CHASS_ABNOR_TEST
    if (0xFF != __chass_abnor_data.axle_index && __chass_abnor_data.axle_index == axle_inc) {
        diagnosis_publish_msg(__chass_abnor_data.axle_data);

        axle_inc++;
        if (axle_inc >= 4) {
            axle_inc = 0;
        }

        return;
    }
#endif

    switch (axle_inc) {
    case 0:
        axle_p = &leftFront;
        break;
    case 1:
        axle_p = &rightFront;
        break;
    case 2:
        axle_p = &leftRear;
        break;
    case 3:
        axle_p = &rightRear;
        break;
    default:
        axle_inc = 0;
        break;
    };

    axle_inc++;
    if (axle_inc >= 4)
        axle_inc = 0;
    tinyros::atris_msgs::chassis_abnor_info msgs;

    strcpy(msgs.axle, axle_p->name);
    msgs.chassis_abnor = 0;
    msgs.direct_error_code = axle_p->drive_error_code;
    msgs.steer_error_code = axle_p->steering_error_code;

    msgs.direct_temperature = axle_p->drive_motor_temperature;
    msgs.steer_temperature = axle_p->steering_motor_temperature;

    msgs.direct_torque = axle_p->currentDrivetorque;
    msgs.steer_torque = axle_p->currentSteeringtorque;

    msgs.direct_speed = axle_p->currentSteeringtorque;
    msgs.steer_angle = wheelTheta(axle_p);
    msgs.steer_485_angle = wheelTheta485(axle_p);
    msgs.direct_speed = wheelSpeedTrans(axle_p);
    diagnosis_publish_msg(msgs);
}

/*send udock data*/
static void udock_status_post(void) {
    uint8_t temp = 0;
    tinyros::atris_msgs::CanPkg msg;

    msg.cmd = CANPKG_CMD_UDOCK_STATUS_REPORT;

    temp = udock_get_status_report();
    if (temp) {
        msg.data_i[0] = temp;
        msg.data_i[1] = 0;
        msg.data_i[2] = 0;
        canpkg_publish_msg(msg);
//		LOG_W("send udock status to host: %d\n", temp);
    }
}

/*send voltage data*/
static void robot_mode_post(void) {
    tinyros::atris_msgs::CanPkg msg;

    msg.cmd = CANPKG_CMD_ROBOT_MODE_REQUEST;
    msg.data_i[0] = robot_mode_get();

    canpkg_publish_msg(msg);
}
/*send voltage data*/
static void voltage_data_post(void) {
    tinyros::atris_msgs::CanPkg msg;

    msg.cmd = CANPKG_CMD_VOLTAGE_DET_DATA_REPORT;
    msg.data_i[0] = vol_get_data(0);
    msg.data_i[1] = vol_get_data(1);
    msg.data_i[2] = vol_get_data(2);
    msg.data_i[3] = vol_get_data(3);
    msg.data_i[4] = vol_get_data(4);
    msg.data_i[5] = vol_get_data(5);

    canpkg_publish_msg(msg);
}

/*send current data*/
static void current_data_post(void) {
    tinyros::atris_msgs::CanPkg msg;

    msg.cmd = CANPKG_CMD_CURRENT_DET_DATA_REPORT;
    msg.data_i[0] = curr_get_data(0);
    msg.data_i[1] = curr_get_data(1);
    msg.data_i[2] = curr_get_data(2);
    msg.data_i[3] = curr_get_data(3);

    canpkg_publish_msg(msg);
}

static uint8_t remote_report_permission(void) {
    if ((remote_is_effecting() == NO)
    /*
     //		(remote_get_mode() == REMOTE_SAFE) ||\
//       (anticollision_get_status() !=0)   ||\
//       (bat_is_inCharging() == YES)       ||\
//       (charge_in_status() == CHARGE_STATUS_IN)\*/
    ) {
        return NO;
    } else {
        return YES;
    }
}
/*send remote data*/
static void remote_data_post(void) {
    uint16_t remote_sbus_channels[16] = { 0 };
    uint8_t remote_sbus_flag;
    uint8_t i;
    tinyros::atris_msgs::CanPkg msg;

    remote_sbus_flag = remote_get_sbus_data(remote_sbus_channels);

    msg.cmd = CANPKG_CMD_REMOTE_INFO_REPORT;
    msg.data_i[0] = remote_sbus_flag;

    for (i = 0; i < 16; i++) {
        msg.data_i[i + 1] = remote_sbus_channels[i];
    }

    canpkg_publish_msg(msg);
}

/*send anti and estop data*/
static void anti_data_post(void) {
    tinyros::atris_msgs::CanPkg msg;

    msg.cmd = CANPKG_CMD_ANTI_INFO_REPORT;
    msg.data_i[0] = e_stop_get_status();
    msg.data_i[1] = anti_get_data();

    canpkg_publish_msg(msg);
//	LOG_W("chassis anti-data: %d, %d\n", msg.data_i[0], msg.data_i[1]);
}

/*send NTC and hum_chm data*/
static void ntc_hum_chm_data_post(void) {
    int8_t ntc_temp[4] = { 0 };
    tinyros::atris_msgs::CanPkg msg;

    ntc_get_temp_data(ntc_temp);
    msg.cmd = CANPKG_CMD_HSU_CHM_INFO_REPORT;
    msg.data_i[0] = sensor_get_hsu_temp();
    msg.data_i[1] = sensor_get_hsu_hum();
    msg.data_i[2] = ntc_temp[0];
    msg.data_i[3] = ntc_temp[1];
    msg.data_i[4] = ntc_temp[2];
    msg.data_i[5] = ntc_temp[3];

    canpkg_publish_msg(msg);
}

/*send brake data*/
static void brake_data_post(void) {
    brake_t brake_status;
    tinyros::atris_msgs::CanPkg msg;

    brake_get_status(&brake_status);
    msg.cmd = CANPKG_CMD_BRAKE_STATUS_REPORT;
    msg.data_i[0] = brake_status.status;
    msg.data_i[1] = brake_status.cause;

    canpkg_publish_msg(msg);
}
#if 0
/*send bms_hw_ver data*/
static void bms_hw_ver_post(void)
{
    char version[80];
	char* ptr = version;
    tinyros::atris_msgs::CanPkg tx_msg;
	
	LOG_W("host ask the bms hw ver.");
		
    tx_msg.cmd = CANPKG_CMD_BMS_HVER_ACK;
	get_bms_hw_version(version);
	tx_msg.data_s = version;

    canpkg_publish_msg(tx_msg);
}

/*send bms_sw_ver data*/
static void bms_sw_ver_post(void)
{
    char version[80];
	char* ptr = version;
    tinyros::atris_msgs::CanPkg tx_msg;
	
	LOG_W("host ask the bms sw ver.");
		
    tx_msg.cmd = CANPKG_CMD_BMS_SVER_ACK;
	get_bms_sw_version(version);
	tx_msg.data_s = version;

    canpkg_publish_msg(tx_msg);
}
#endif
static int32_t msg_bat_log_callback(const tinyros::atris_msgs::CanPkg &msg) {
    tinyros::atris_msgs::CanPkg tx_msg;

    tx_msg.cmd = CANPKG_CMD_BATTERY_LOG_REPORT;
    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_power_onoff_callback(const tinyros::atris_msgs::CanPkg &msg) {
    if (msg.data_i[0]) {
        power_down_clr_status_report(msg.data_i[0]);
    }
    return 0;
}

static int32_t msg_power_shutdown_callback(const tinyros::atris_msgs::CanPkg &msg) {
    tinyros::atris_msgs::CanPkg tx_msg;

//	LOG_W("canpkg_rec data.\n");
    if (msg.data_i[0] == 1) {
        LOG_W("host request to start the normal-shutdown process.");
        power_shutdown_set_by_host();
    } else if (msg.data_i[0] == 2) {
        LOG_W("host request to start the udock-standby process.");
        power_standby_set();
    } else if (msg.data_i[0] == 3) {
        LOG_W("host request to start the standby process.");
        light_standby_set();
        power_standby_set();
    } else if (msg.data_i[0] == 4) {
        LOG_W("host request to wake up the system from the standby process.");
        power_wakeup_set();
    } else if (msg.data_i[0] == 5) {
        LOG_W("host request to start the manual charge standby process.");
        power_standby_set();
    }

    tx_msg.cmd = CANPKG_CMD_POWER_SHUTDOWN_REQUEST_ACK;
    tx_msg.data_i[0] = msg.data_i[0];
    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_udock_pointdown_callback(const tinyros::atris_msgs::CanPkg &msg) {
    tinyros::atris_msgs::CanPkg tx_msg;

    if (msg.data_i[0] == 1) {
        udock_set_pointdown_flag();
        LOG_W("info mcu to start pointdown.");
    }

    tx_msg.cmd = CANPKG_CMD_UDOCK_POINT_DOWN_ACK;
    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_udock_statusreport_callback(const tinyros::atris_msgs::CanPkg &msg) {

    if (msg.data_i[0] == 0) {
        udock_clr_pole_report();
        LOG_W("host ack the cmd of charge pole.");
    } else if (msg.data_i[0] == 1) {
        udock_clr_pointdown_finish();
        LOG_W("host ack the cmd of point down finish.");
    }

    return 0;
}

static int32_t msg_udock_pointup_callback(const tinyros::atris_msgs::CanPkg &msg) {
    tinyros::atris_msgs::CanPkg tx_msg;

    if (msg.data_i[0] == 1) {
        udock_set_pointup_finish();
        LOG_W("host pointup success.");
    } else {
        LOG_W("host pointup fail.");
    }

    tx_msg.cmd = CANPKG_CMD_UDOCK_POINT_UP_ACK;
    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_power_status_callback(const tinyros::atris_msgs::CanPkg &msg) {
    tinyros::atris_msgs::CanPkg tx_msg;

    if (msg.data_i[0] == 1) {
        LOG_W("host ask the state of the power.");
    } else if (msg.data_i[0] == 2) {
        LOG_W("host set the state of the power.");
        power_set_status(msg.data_i[1]);
    }
    tx_msg.cmd = CANPKG_CMD_POWER_STATUS_REPORT;
    tx_msg.data_i[0] = power_get_status();

    return canpkg_publish_msg(tx_msg);
}
//static int32_t msg_fan_status_callback(const tinyros::atris_msgs::CanPkg &msg)
//{
//    tinyros::atris_msgs::CanPkg tx_msg;

//    tx_msg.cmd = CANPKG_CMD_FAN_STATUS_REPORT;
//    return canpkg_publish_msg(tx_msg);
//}

static int32_t msg_brake_set_callback(const tinyros::atris_msgs::CanPkg &msg) {
    brake_t brake_status;
    tinyros::atris_msgs::CanPkg tx_msg;

    if (msg.data_i[0] == 1) {
        brake_set_byhost(msg.data_i[1]);
        LOG_W("host set the brake: %d\n", msg.data_i[1]);
    }
    tx_msg.cmd = CANPKG_CMD_BRAKE_STATUS_ACK;
    brake_get_status(&brake_status);
    tx_msg.data_i[0] = brake_status.status;

    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_transport_set_callback(const tinyros::atris_msgs::CanPkg &msg) {
    tinyros::atris_msgs::CanPkg tx_msg;

    chassis_transport_mode_set(msg.data_i[0]);
    LOG_W("host set the transport mode: %d\n", msg.data_i[0]);

    tx_msg.cmd = CANPKG_CMD_TRANSPORT_STATUS_ACK;
    tx_msg.data_i[0] = chassis_transport_mode_get();

    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_mcu_ver_callback(const tinyros::atris_msgs::CanPkg &msg) {
    tinyros::atris_msgs::CanPkg tx_msg;
    fota_flags_t *pflags = fota_flags_get();

    LOG_W("host ask the mcu ver.");

    tx_msg.cmd = CANPKG_CMD_MCU_VER_ACK;
    tx_msg.data_i[0] = core_hw_version_get();
    tx_msg.data_i[1] = board_hw_version_get();
    tx_msg.data_s = pflags->app_version;
    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_bms_hw_ver_callback(const tinyros::atris_msgs::CanPkg &msg) {
    char version[80];
    tinyros::atris_msgs::CanPkg tx_msg;

    LOG_W("host ask the bms hw ver.");

    tx_msg.cmd = CANPKG_CMD_BMS_HVER_ACK;
    get_bms_hw_version(version);
    tx_msg.data_s = version;

    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_bms_sw_ver_callback(const tinyros::atris_msgs::CanPkg &msg) {
    char version[80];
    tinyros::atris_msgs::CanPkg tx_msg;

    LOG_W("host ask the bms sw ver.");

    tx_msg.cmd = CANPKG_CMD_BMS_SVER_ACK;
    get_bms_sw_version(version);
    tx_msg.data_s = version;

    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_drop_prevent_callback(const tinyros::atris_msgs::CanPkg &msg) {
    static uint8_t cn_no_cliff = 0;
    /*  disable the cliff detection function */

    if (cliff_detect_flag != 0) {
        if (msg.data_i[0] == 0) {

            if (cn_no_cliff >= 3) {
                brake_set(BRAKE_CAUSE_CLIFF, BRAKE_STATUS_UNLOCK);
                cn_no_cliff = 3;
            } else {
                cn_no_cliff++;
            }
        } else {
            cn_no_cliff = 0;
            LOG_W("detect the cliff:               %d\n",msg.data_i[0]);
            brake_set(BRAKE_CAUSE_CLIFF, BRAKE_STATUS_LOCK);
        }
    }
    return 0;
}

static void canpkg_app_task_main(void *_param) {
    uint32_t s_timer_50ms = 0;
    uint32_t s_timer_200ms = 0;
    uint32_t s_timer_1000ms = 0;

    canpkg_subscribe_cb_add(CANPKG_CMD_BATTERY_LOG_ASK, msg_bat_log_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_POWER_ONOFF_REQUEST_ACK, msg_power_onoff_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_POWER_SHUTDOWN_REQUEST, msg_power_shutdown_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_UDOCK_POINT_DOWN, msg_udock_pointdown_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_UDOCK_STATUS_REPORT_ACK, msg_udock_statusreport_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_UDOCK_POINT_UP, msg_udock_pointup_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_POWER_STATUS_ASK_SET, msg_power_status_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_BRAKE_STATUS_SET, msg_brake_set_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_TRANSPORT_STATUS_SET, msg_transport_set_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_MCU_VER_ASK, msg_mcu_ver_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_BMS_HVER_ASK, msg_bms_hw_ver_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_BMS_SVER_ASK, msg_bms_sw_ver_callback);

    monitor_canpkg_subscribe_cb_add(CANPKG_CMD_DROP_PREVENT_INFO_REPORT, msg_drop_prevent_callback);

    while (1) {
        rtc_sync();

        udock_status_post();
        power_down_request_post();

        if (os_gettime_ms() - s_timer_50ms >= 50) {
            s_timer_50ms = os_gettime_ms();

            if ((remote_report_permission()) != 0) {
                remote_data_post();
            }
        } else if (os_gettime_ms() < s_timer_50ms) {
            s_timer_50ms = os_gettime_ms();
        }

        if (os_gettime_ms() - s_timer_200ms >= 200) {
            s_timer_200ms = os_gettime_ms();
            brake_data_post();
            chassis_diagnosis_post();
        } else if (os_gettime_ms() < s_timer_200ms) {
            s_timer_200ms = os_gettime_ms();
        }

        if (os_gettime_ms() - s_timer_1000ms >= 1000) {
            s_timer_1000ms = os_gettime_ms();
            bat_data_post();
            voltage_data_post();
            current_data_post();
            anti_data_post();
            ntc_hum_chm_data_post();
            light_mode_request_post();
            robot_mode_post();
        } else if (os_gettime_ms() < s_timer_1000ms) {
            s_timer_1000ms = os_gettime_ms();
        }

        rt_thread_mdelay(10);
    }
}

int32_t canpkg_app_init(void) {
    if (DF_THREAD_STATIC_MEMORY == 0) {
        rt_thread_t thread = rt_thread_create("task_canpkg_app", canpkg_app_task_main,
        RT_NULL,
        TASK_STACK_SIZE_CANPKG_APP,
        TASK_PRIORITY_CANPKG_APP, 20);
        if (thread != RT_NULL) {
            rt_thread_startup(thread);
        } else {
            return -1;
        }
    } else {
        static struct rt_thread canpkg_app_thread;
        ALIGN(RT_ALIGN_SIZE)
        static char canpkg_app_thread_stack[TASK_STACK_SIZE_CANPKG_APP];
        rt_err_t result = RT_EOK;
        result = rt_thread_init(&canpkg_app_thread, "task_canpkg_app", canpkg_app_task_main, RT_NULL,
                &canpkg_app_thread_stack[0], sizeof(canpkg_app_thread_stack),
                TASK_PRIORITY_CANPKG_APP, 20);

        if (result == RT_EOK) {
            rt_thread_startup(&canpkg_app_thread);
            return 0;
        } else {

            LOG_I("%s thread create failed.",__FUNCTION__);
            return -1;
        }

    }
    return 0;
}

/******************************************************************************************************************/
static void Var_EnalbeDisableCliff(uint8_t argc, char **argv) {
    if (argc < 2 || argc > 3) {
        rt_kprintf("Please input: Var Enable Disable <1/0> 1:enable 0:disable\n  cliff_detect_flag  \n");
    } else {
        if ((!strncmp("cliff_detect", argv[1], 12))) {
            cliff_detect_flag = atoi(argv[2]);
        }

    }
}
MSH_CMD_EXPORT(Var_EnalbeDisableCliff, Variable Enable Disable);

