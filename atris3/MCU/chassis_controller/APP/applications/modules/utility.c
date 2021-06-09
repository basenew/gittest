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
#include "e_stop_detect.h"
#include "display.h"
#include "hsu_chm_01a.h"
#include "brake.h"

#define LOG_TAG              "remote"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>

extern uint32_t g_system_startup_moment;


//在这里放不会堵塞的loop
static void utility_task_main(void* _param)
{
    uint32_t s_timer_10ms   = 0;
    uint32_t s_timer_20ms   = 0;
    uint32_t s_timer_50ms   = 0;
    uint32_t s_timer_100ms  = 0;
    uint32_t s_timer_200ms  = 0;
    uint32_t s_timer_500ms  = 0;
    uint32_t s_timer_1000ms = 0;
    uint32_t s_timer_2000ms = 0;
    
//    uint8_t  s_timerlater_flag_10000ms = 0; //第一次松开刹车
//	uint8_t  s_brake_report_en = 0;
//	static uint8_t power_on_key = 0;
//	static uint8_t estop_key = 0;
    
    while(1)
    {
        if(os_gettime_ms() - s_timer_10ms >= 10) 
        {
            s_timer_10ms = os_gettime_ms();
			e_stop_detect();
			anti_brake_process();
        } else if (os_gettime_ms() < s_timer_10ms) {
            s_timer_10ms = os_gettime_ms();
        }
		
        if(os_gettime_ms() - s_timer_20ms >= 20) 
        {
            s_timer_20ms = os_gettime_ms();
        } else if (os_gettime_ms() < s_timer_20ms) {
            s_timer_20ms = os_gettime_ms();
        }
             
        if(os_gettime_ms() - s_timer_50ms >= 50) 
        {
            s_timer_50ms = os_gettime_ms();
        } else if (os_gettime_ms() < s_timer_50ms) {
            s_timer_50ms = os_gettime_ms();
        }
        
        if(os_gettime_ms() - s_timer_100ms >= 200) 
        {
            s_timer_100ms = os_gettime_ms();
        } else if (os_gettime_ms() < s_timer_100ms) {
            s_timer_100ms = os_gettime_ms();
        }
        
        if(os_gettime_ms() - s_timer_200ms >= 200) 
        {
            s_timer_200ms = os_gettime_ms();
            display_loop();
        } else if (os_gettime_ms() < s_timer_200ms) {
            s_timer_200ms = os_gettime_ms();
        }
        
        if(os_gettime_ms() - s_timer_500ms >= 500) 
        {
            s_timer_500ms = os_gettime_ms();

        } else if (os_gettime_ms() < s_timer_500ms) {
            s_timer_500ms = os_gettime_ms();
        }
        
        if(os_gettime_ms() - s_timer_1000ms >= 1000)
        {
            s_timer_1000ms = os_gettime_ms();
        } else if (os_gettime_ms() < s_timer_1000ms) {
            s_timer_1000ms = os_gettime_ms();
        }
        
        if(os_gettime_ms() - s_timer_2000ms >= 2000) 
        {
            s_timer_2000ms = os_gettime_ms();

        } else if (os_gettime_ms() < s_timer_2000ms) {
            s_timer_2000ms = os_gettime_ms();
        }
		
        rt_thread_mdelay(10);
    }
}

int32_t utility_init(void)
{
	e_stop_init();
    display_init();
if(DF_THREAD_STATIC_MEMORY == 0){	
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
}else{
    static struct rt_thread utility_thread;
    ALIGN(RT_ALIGN_SIZE)
    static char utility_thread_stack[TASK_STACK_SIZE_UTILITY]; 
    	rt_err_t result = RT_EOK;
    result = rt_thread_init(&utility_thread,
                            "utility",
                            utility_task_main, RT_NULL,
                            &utility_thread_stack[0], sizeof(utility_thread_stack),
                            TASK_PRIORITY_UTILITY, 20);

    if (result == RT_EOK){
    	rt_thread_startup(&utility_thread);
        return 0;
    }else {
        LOG_I("%s thread create failed.",__FUNCTION__);
        return -1;
    }
    	
    
}    
    return 0;
}

