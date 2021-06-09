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
#include "ntc_sample.h"
#include "power_ctrl.h"
#include "voltage_detect.h"
#include "sensor_detec.h"
#include "version.h"
#include "light_effect.h"
#include "sonar.h"
#include "fan.h"
#include "rt_fota.h"
#include "tiny_ros/ros/time.h"
#include "curr_sample.h"

#define LOG_TAG              "canpkg_app"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>


static void rtc_sync_to(void)
{
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

static void rtc_sync(void)
{
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


/*send voltage data*/
static void voltage_data_post(void)
{
    tinyros::atris_msgs::CanPkg msg;
	
    msg.cmd = CANPKG_CMD_VOLTAGE_DET_DATA_REPORT;
    msg.data_i[0] = vol_get_data(0);
	msg.data_i[1] = vol_get_data(1);
	msg.data_i[2] = vol_get_data(2);
	msg.data_i[3] = vol_get_data(3);
	msg.data_i[4] = vol_get_data(4);
	msg.data_i[5] = vol_get_data(5);
	msg.data_i[6] = vol_get_data(6);
	msg.data_i[7] = vol_get_data(7);
	msg.data_i[8] = vol_get_data(8);
	msg.data_i[9] = vol_get_data(9);
	msg.data_i[10] = vol_get_data(10);
	msg.data_i[11] = vol_get_data(11);
	msg.data_i[12] = vol_get_data(12);
	msg.data_i[13] = vol_get_data(13);

    canpkg_publish_msg(msg);
}

/*send current data*/
static void current_data_post(void)
{
    tinyros::atris_msgs::CanPkg msg;
	
    msg.cmd = CANPKG_CMD_CURRENT_DET_DATA_REPORT;
    msg.data_i[0] = curr_get_data(0);
	msg.data_i[1] = curr_get_data(1);

    canpkg_publish_msg(msg);
}

/*send sonar data*/
static void sonar_data_post(void)
{
	uint16_t sonar_data[4] = {0};
    tinyros::atris_msgs::CanPkg msg;
	
	sonar_get_data(sonar_data);
    msg.cmd = CANPKG_CMD_SONAR_DATA_REPORT;
	msg.data_i[0] = sonar_data[0];
	msg.data_i[1] = sonar_data[1];
	msg.data_i[2] = sonar_data[2];
	msg.data_i[3] = sonar_data[3];

    canpkg_publish_msg(msg);
}

/*send drop_prevent sensor data*/
static void drop_prevent_data_post(void)
{
    tinyros::atris_msgs::CanPkg msg;
	
    msg.cmd = CANPKG_CMD_DROP_PREVENT_INFO_REPORT;
	msg.data_i[0] = sensor_get_dt35();

    canpkg_publish_msg(msg);
}

/*send NTC and hum_chm data*/
static void ntc_hum_chm_data_post(void)
{
	int8_t ntc_temp[4] = {0};
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

#if 0
/*send light data*/
static void light_data_post(void)
{
	light_status_t* _ptr;
    tinyros::atris_msgs::CanPkg msg;
	
	_ptr = light_remote_get();
    msg.cmd = CANPKG_CMD_LIGHT_ACK;
	msg.data_i[0] = _ptr->action;
	msg.data_i[1] = _ptr->color;
	msg.data_i[2] = _ptr->lum_max;

    canpkg_publish_msg(msg);
}

static void fan_status_post(void)
{
	fan_status_t fan_status;
	
    tinyros::atris_msgs::CanPkg tx_msg;

	fan_status_get(&fan_status);
    tx_msg.cmd = CANPKG_CMD_FAN_STATUS_REPORT;
		tx_msg.data_i[0] = fan_status.speed_data;
		tx_msg.data_i[1] = fan_status.err_data;
    canpkg_publish_msg(tx_msg);
}
#endif

static int32_t msg_light_status_callback(const tinyros::atris_msgs::CanPkg &msg)
{
	light_status_t light_status_host;
	light_status_t* _ptr;
    tinyros::atris_msgs::CanPkg tx_msg;
	
	if(msg.data_i[0] == 0)
	{
		LOG_W("host ask the state of the light.");
	}
	else if(msg.data_i[0] == 1)
	{
		if(msg.data_i[4] == DF_ENABLE)
		{
			LOG_W("host set the state of the light.");
			light_status_host.action = msg.data_i[1];
			light_status_host.color = (light_color_t)msg.data_i[2];
			light_status_host.lum_max = msg.data_i[3];
			light_status_host.ctr_right = msg.data_i[4];
			light_remote_set(light_status_host);
		}
		else if(msg.data_i[4] == DF_DISABLE)
		{
			LOG_W("host release the ctr_right of the light.");
			light_status_host.action = msg.data_i[1];
			light_status_host.color = (light_color_t)msg.data_i[2];
			light_status_host.lum_max = msg.data_i[3];
			light_status_host.ctr_right = msg.data_i[4];
			light_remote_set(light_status_host);
		}
	}
	_ptr = light_remote_get();
    tx_msg.cmd = CANPKG_CMD_LIGHT_ACK;
	tx_msg.data_i[0] = _ptr->action;
	tx_msg.data_i[1] = _ptr->color;
	tx_msg.data_i[2] = _ptr->lum_max;

    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_power_status_callback(const tinyros::atris_msgs::CanPkg &msg)
{
    tinyros::atris_msgs::CanPkg tx_msg;
	
	if(msg.data_i[0] == 1)
	{
		LOG_W("host ask the state of the power.");
		tx_msg.data_i[0] = power_get_status();
	}
	else if(msg.data_i[0] == 2)
	{
		LOG_W("host set the state of the power.");
		power_set_status(msg.data_i[1]);
	}
    tx_msg.cmd = CANPKG_CMD_POWER_STATUS_REPORT;
	tx_msg.data_i[0] = power_get_status();

    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_sonar_ver_callback(const tinyros::atris_msgs::CanPkg &msg)
{
    tinyros::atris_msgs::CanPkg tx_msg;
	
    tx_msg.cmd = CANPKG_CMD_SONAR_VER_ACK;

    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_fan_status_callback(const tinyros::atris_msgs::CanPkg &msg)
{
	fan_status_t fan_status;
	
    tinyros::atris_msgs::CanPkg tx_msg;

	if(msg.data_i[0] == 0)
	{
		LOG_W("host ask the state of the fan.");
	}
	else if(msg.data_i[0] == 1)
	{
		LOG_W("host set the state of the fan.");
		fan_remote_set(msg.data_i[1]);
	}
	
	fan_status_get(&fan_status);
    tx_msg.cmd = CANPKG_CMD_FAN_STATUS_REPORT;
		tx_msg.data_i[0] = fan_status.speed_data;
		tx_msg.data_i[1] = fan_status.err_data;
    return canpkg_publish_msg(tx_msg);
}


static int32_t msg_mcu_ver_callback(const tinyros::atris_msgs::CanPkg &msg)
{
    tinyros::atris_msgs::CanPkg tx_msg;
    fota_flags_t* pflags = fota_flags_get();
	
	LOG_W("host ask the mcu ver.");
		
    tx_msg.cmd = CANPKG_CMD_MCU_VER_ACK;
	tx_msg.data_i[0] = core_hw_version_get();
	tx_msg.data_i[1] = board_hw_version_get();
	tx_msg.data_s = pflags->app_version;
    return canpkg_publish_msg(tx_msg);
}

static int32_t msg_chassis_bat_callback(const tinyros::atris_msgs::CanPkg &msg)
{
	if(msg.data_i[24] == 0x01)	//charging
	{
		if((light_get_evt_bits() & (1<<LIGHT_EVT_BAT_CHARGE_ING)) == 0)
		{
			light_post_event(LIGHT_EVT_BAT_CHARGE_ING, 1);
			LOG_W("bat charging, light enable.");
		}
	}
	else if(msg.data_i[24] == 0x02)	//charge full
	{
		if((light_get_evt_bits() & (1<<LIGHT_EVT_BAT_CHARGE_FULL)) == 0)
		{
			light_post_event(LIGHT_EVT_BAT_CHARGE_ING, 0);
			light_post_event(LIGHT_EVT_BAT_CHARGE_FULL, 1);
			LOG_W("bat charg-full, light enable.");
		}
	}
	else
	{
		if((light_get_evt_bits() & (1<<LIGHT_EVT_BAT_CHARGE_ING)) != 0)
		{
			light_post_event(LIGHT_EVT_BAT_CHARGE_ING, 0);
			LOG_W("bat no charging, light enable.");
		}
		
		if((light_get_evt_bits() & (1<<LIGHT_EVT_BAT_CHARGE_FULL)) != 0)
		{
			light_post_event(LIGHT_EVT_BAT_CHARGE_FULL, 0);
			LOG_W("bat no charg-full, light enable.");
		}
	}
//	LOG_W("rec chassis data: %d, %d\n", msg.data_i[0], msg.data_i[1]);
	
	return 0;
		
}

static int32_t msg_chassis_anti_callback(const tinyros::atris_msgs::CanPkg &msg)
{
	/*if(msg.data_i[0] == 1)	//e_stop trige
	{
		if((light_get_evt_bits() & (1<<LIGHT_EVT_WARN)) == 0)
		{
			light_post_event(LIGHT_EVT_WARN, 1);
			LOG_W("e-stop tiger, light enable.");
		}
	}
	else
	{
		if((light_get_evt_bits() & (1<<LIGHT_EVT_WARN)) != 0)
		{
			light_post_event(LIGHT_EVT_WARN, 0);
			LOG_W("e-stop release, light enable.");
		}
	}*/
	
//	LOG_W("rec chassis data: %d, %d\n", msg.data_i[0], msg.data_i[1]);
	
	return 0;
		
}


static int32_t msg_chassis_brake_callback(const tinyros::atris_msgs::CanPkg &msg)
{
	if((msg.data_i[0] == 1) && (msg.data_i[1]&0x06))	//e_stop trige
	{
		if((light_get_evt_bits() & (1<<LIGHT_EVT_WARN)) == 0)
		{
			light_post_event(LIGHT_EVT_WARN, 1);
			LOG_W("e-stop tiger, light enable.");
		}
	}
	else
	{
		if((light_get_evt_bits() & (1<<LIGHT_EVT_WARN)) != 0)
		{
			light_post_event(LIGHT_EVT_WARN, 0);
			LOG_W("e-stop release, light enable.");
		}
	}
	
//	LOG_W("rec chassis data: %d, %d\n", msg.data_i[0], msg.data_i[1]);
	
	return 0;
		
}

//static uint8_t s_power_standby_flag = 0;
static uint8_t g_power_mode = POWRE_WORK;
static int32_t msg_chassis_light_mode_callback(const tinyros::atris_msgs::CanPkg &msg)
{
	if(msg.data_i[0] == LIGHT_STANDBY)	//light standby
	{
		if((light_get_evt_bits() & (1<<LIGHT_EVT_STANDBY)) == 0)
		{
			light_post_event(LIGHT_EVT_STANDBY, 1);
			LOG_W("light_mode:standby enable.");
		}
	}
	else	//WORK_MODE
	{
		if((light_get_evt_bits() & (1<<LIGHT_EVT_STANDBY)) != 0)
		{
			light_post_event(LIGHT_EVT_STANDBY, 0);
			LOG_W("light_mode:standby disable.");
		}
	}
	
	if(msg.data_i[1] != g_power_mode)
	{
		g_power_mode = msg.data_i[1];
		if(g_power_mode == POWER_STANDBY)
		{
			LOG_W("power_mode:standby enable.");
			power_standby_status();
		}
		else	//work mode
		{
			LOG_W("power_mode:standby disable.");
			power_work_status();
		}
	}
	
//	LOG_W("rec chassis data: %d, %d\n", msg.data_i[0], msg.data_i[1]);
	
	return 0;
		
}

static void canpkg_app_task_main(void* _param)
{
//    uint32_t s_timer_10ms   = 0;
//    uint32_t s_timer_20ms   = 0;
    uint32_t s_timer_150ms   = 0;
//    uint32_t s_timer_200ms  = 0;
//    uint32_t s_timer_500ms  = 0;
    uint32_t s_timer_1000ms = 0;
//    uint32_t s_timer_2000ms = 0;
    
    canpkg_subscribe_cb_add(CANPKG_CMD_LIGHT_ASK_SET, msg_light_status_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_POWER_STATUS_ASK_SET, msg_power_status_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_SONAR_VER_ASK, msg_sonar_ver_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_FAN_STATUS_ASK_SET, msg_fan_status_callback);
    canpkg_subscribe_cb_add(CANPKG_CMD_MCU_VER_ASK, msg_mcu_ver_callback);
	
    chassis_canpkg_subscribe_cb_add(CANPKG_CMD_BATTERY_INFO_REPORT, msg_chassis_bat_callback);
    chassis_canpkg_subscribe_cb_add(CANPKG_CMD_LIGHT_MODE_REQUEST, msg_chassis_light_mode_callback);
    chassis_canpkg_subscribe_cb_add(CHASSIS_CANPKG_CMD_ANTI_INFO_REPORT, msg_chassis_anti_callback);
    chassis_canpkg_subscribe_cb_add(CANPKG_CMD_BRAKE_STATUS_REPORT, msg_chassis_brake_callback);
    
    while(1)
    {
		rtc_sync();
		
             
        if(os_gettime_ms() - s_timer_150ms >= 150) 
        {
            s_timer_150ms = os_gettime_ms();
			sonar_data_post();
			drop_prevent_data_post();
        } else if (os_gettime_ms() < s_timer_150ms) {
            s_timer_150ms = os_gettime_ms();
        }
        
        if(os_gettime_ms() - s_timer_1000ms >= 1000)
        {
            s_timer_1000ms = os_gettime_ms();
			voltage_data_post();
			current_data_post();
			ntc_hum_chm_data_post();
        } else if (os_gettime_ms() < s_timer_1000ms) {
            s_timer_1000ms = os_gettime_ms();
        }
		
        rt_thread_mdelay(50);
    }
}

int32_t canpkg_app_init(void)
{
	
    rt_thread_t thread = rt_thread_create("task_canpkg_app", \
                                  canpkg_app_task_main, \
                                  RT_NULL, \
                                  TASK_STACK_SIZE_CANPKG_APP, \
                                  TASK_PRIORITY_CANPKG_APP, \
                                  20);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        return -1;
    }
    
    return 0;
}

uint8_t power_mode_get(void) 
{
	return g_power_mode;
}












