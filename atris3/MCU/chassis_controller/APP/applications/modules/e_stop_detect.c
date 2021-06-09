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
#include "e_stop_detect.h"
#include "brake.h"


#define LOG_TAG              "e_stop_detect"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>



#define   PIN_E_STOP_DET_PH10                     GET_PIN(H, 10)


static uint8_t g_e_stop_value = 0;

/**
 * 急停信号发生检测
 * @param      : void
 * @return     : void
 */
void e_stop_detect(void)
{
	uint8_t e_stop_new_value = 0;
	static uint8_t  e_stop_det_cnt = 0;

	if(rt_pin_read(PIN_E_STOP_DET_PH10) == PIN_HIGH)
	{
		e_stop_new_value |= 0x01;
	}
	
	
	if(e_stop_new_value != g_e_stop_value)
	{
		if(e_stop_det_cnt > 5)
		{
			e_stop_det_cnt = 0;
			g_e_stop_value = e_stop_new_value;
			
			if(g_e_stop_value)
			{
                LOG_W("e-stop trige");
				brake_set(BRAKE_CAUSE_E_STOP, BRAKE_STATUS_LOCK);
			}
			else
			{
                LOG_W("e-stop release");
				brake_unlock_byestop();
//				brake_set(BRAKE_CAUSE_E_STOP, BRAKE_STATUS_UNLOCK);
			}

			
		}
		else
		{
			e_stop_det_cnt ++;
		}
	}
	
    return;
}


int32_t e_stop_init(void)
{
    rt_pin_mode(PIN_E_STOP_DET_PH10,          PIN_MODE_INPUT);

    
    return 0;
}

uint8_t e_stop_get_status(void)
{
	return g_e_stop_value;
}


//--------------------------------------------------------------------------------------------------------------
#include "finsh.h"




