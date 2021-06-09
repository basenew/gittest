/*
 * COPYRIGHT (C) Copyright 2012-2017; UBT TECH; SHENZHEN, CHINA
 *
 * File       : power_manager.c
 * Brief      : 电源控制管理

 * Change Logs
 * Date           Author        Version       Notes
 * 2018-12-18     wuxiaofeng    v1.0          
 *
 */
#include "common.h"
#include "brake.h"
#include "e_stop_detect.h"
#include "ntc_sample.h"


#define LOG_TAG              "brake"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>

static brake_t g_brake;
static uint32_t g_brake_release_time;

static uint8_t g_chassis_rtransport_mode = DF_DISABLE;


void brake_get_status(brake_t* _brake)
{
	_brake->cause = g_brake.cause;
	_brake->status = g_brake.status;
}
uint8_t get_brake_status(void)
{
	return g_brake.status;
}

uint8_t get_brake_cause(void)
{
	return g_brake.cause;
}

void brake_set(uint8_t cause, uint8_t _status)
{
    if(g_brake.lock != RT_NULL) {
        rt_mutex_take(g_brake.lock, RT_WAITING_FOREVER);
    }

    if (_status == BRAKE_STATUS_UNLOCK)
    {
		g_brake.cause = 0;
		g_brake.status = _status;
        /*g_brake.cause  &= ~(1<<cause);
		if(g_brake.cause == 0)
		{
			 g_brake.status = _status;
		}
		
		switch(cause)
		{
			case BRAKE_CAUSE_REBOOT:
				LOG_W("unbrake_cause: reboot");
				break;
			
			case BRAKE_CAUSE_ANTI:
				LOG_W("unbrake_cause: anti_front");
				break;
			
			case BRAKE_CAUSE_CHARGEGUN:
				LOG_W("unbrake_cause: charge gun");
				break;
			
			case BRAKE_CAUSE_HOST:
				LOG_W("unbrake_cause: can");
				break;
			
			case BRAKE_CAUSE_E_STOP:
				LOG_W("unbrake_cause: e-stop");
				break;
			
			default :
				LOG_W("unbrake_cause: unkown");
				break;
		}*/
    }
    else
    {
        g_brake.cause |= (1<<cause);
        g_brake.status = _status;
		
		switch(cause)
		{
			case BRAKE_CAUSE_REBOOT:
				LOG_W("brake_cause: reboot");
				break;
			
			case BRAKE_CAUSE_ANTI:
				LOG_W("brake_cause: anti_front");
				break;
			
			case BRAKE_CAUSE_CHARGEGUN:
				LOG_W("brake_cause: charge gun");
				break;
			
			case BRAKE_CAUSE_HOST:
				LOG_W("brake_cause: can");
				break;
			
			case BRAKE_CAUSE_E_STOP:
				LOG_W("brake_cause: e-stop");
				break;
			
			case BRAKE_CAUSE_CLIFF:
				LOG_W("brake_cause: detect the cliff");
				break;
			
			default :
				LOG_W("brake_cause: unkown");
				break;
		}
    }

    if(g_brake.lock != RT_NULL) {
        rt_mutex_release(g_brake.lock);
    }
}

void brake_set_byhost(uint8_t status)
{
    if(status & 0x01)  
    {
        brake_set(BRAKE_CAUSE_HOST, BRAKE_STATUS_LOCK);
    }
    else
    {
		if(e_stop_get_status())	//急停按钮按下时不能解锁刹车,但可以释放除急停按钮外其他刹车原因；
		{
			//上层松开刹车时，判断是不是防撞条触发的
			if(g_brake.cause & (1<<BRAKE_CAUSE_ANTI))
			{
				g_brake_release_time = os_gettime_ms();
			}
			g_brake.cause = (1<<BRAKE_CAUSE_E_STOP);
		}
		else
		{
			//上层松开刹车时，判断是不是防撞条触发的
			if(g_brake.cause & (1<<BRAKE_CAUSE_ANTI))
			{
				g_brake_release_time = os_gettime_ms();
			}
			brake_set(0, BRAKE_STATUS_UNLOCK);
		}
    }
}

void brake_unlock_byestop(void)
{
	g_brake.cause  &= ~(1<<BRAKE_CAUSE_E_STOP);
	if(g_brake.cause == 0)	//除急停外无其他刹车原因，否则仅释放急停刹车原因
	{
		brake_set(0, BRAKE_STATUS_UNLOCK);
	}
}

void anti_brake_process(void)
{
	if(anti_get_data() != 0)
	{
		if(g_brake_release_time != 0 )
		{
			if(os_gettime_ms() - g_brake_release_time >= 30000)
			{
				g_brake_release_time = 0;

				brake_set(BRAKE_CAUSE_ANTI, BRAKE_STATUS_LOCK);
			}
			else if(os_gettime_ms() < g_brake_release_time){
				g_brake_release_time = os_gettime_ms();
			}
		}
	}
	else
	{
		g_brake_release_time = 0;
	}
}

void chassis_transport_mode_set(uint8_t _staus)
{
	if(_staus == DF_ENABLE)
	{
		g_chassis_rtransport_mode = DF_ENABLE;
	}
	else
	{
		g_chassis_rtransport_mode = DF_DISABLE;
	}
}

uint8_t chassis_transport_mode_get(void)
{
	return g_chassis_rtransport_mode;
}

//--------------------------------------------------------------------------------------------------------------
#include "finsh.h"




