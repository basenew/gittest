/*
 * COPYRIGHT (C) Copyright 2012-2017; UBT TECH; SHENZHEN, CHINA
 *
 * File       : utility.c
 * Brief      : 一些简单任务的集中管理

 * Change Logs
 * Date           Author        Version       Notes
 * 2019-07-06     wuxiaofeng    v1.0          
 *
 */
#include "common.h"
#include "app_cfg.h"
#include "board.h"
#include "utility.h"
#include "power_ctrl.h"
#include "msg_canpkg_app.h"

#define LOG_TAG              "utility"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>

uint32_t g_power_up_moment;


static uint8_t  g_spk_open_start = 0; //功放打开标志

void spk_open_start(void)
{
	g_spk_open_start = 0;
	g_power_up_moment = os_gettime_ms();
}
//在这里放不会堵塞的loop
static void utility_task_main(void* _param)
{
//    uint32_t s_timer_10ms   = 0;
    
    
    while(1)
    {
		if(g_spk_open_start == 0)
		{
			if(os_gettime_ms() - g_power_up_moment >= 60000)	//60s
			{
				if(power_mode_get() == POWRE_WORK)
				{
					spk_ctrl_set(DF_ENABLE);
				}
				g_spk_open_start = 1;
			}
		}
//        if(os_gettime_ms() - s_timer_10ms >= 10) 
//        {
//            s_timer_10ms = os_gettime_ms();
//        } else if (os_gettime_ms() < s_timer_10ms) {
//            s_timer_10ms = os_gettime_ms();
//        }
		
        rt_thread_mdelay(100);
    }
}

int32_t utility_init(void)
{
	
    rt_thread_t thread = rt_thread_create("task_utility", \
                                  utility_task_main, \
                                  RT_NULL, \
                                  TASK_STACK_SIZE_UTILITY, \
                                  TASK_PRIORITY_UTILITY, \
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

