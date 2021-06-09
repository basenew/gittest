/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : hw_wdg.c
 * Brief      : 硬件看门狗
 * Note       : 在软件定时器线程和关键线程中喂狗
 * Change Logs
 * Date           Author        Version       Notes
 * 2019-06-18     wuxiaofeng    v1.0
 * 2020-07-02     wuxiaofeng    v1.1          feed it in softtimer
 *
 */
#include "rtthread.h"
#include "board.h"
#include "app_cfg.h"
#include "hw_wdg.h"


#define  WDI_PIN                   GET_PIN(C, 2)
#define  WDI_SET                   rt_pin_write(WDI_PIN, PIN_HIGH)  
#define  WDI_CLR                   rt_pin_write(WDI_PIN, PIN_LOW)  
#define  WDI_STATUS                rt_pin_read(WDI_PIN)    


static volatile uint8_t g_wdg_enable = 0;
static volatile uint8_t g_wdg_inited      = 0;
static struct rt_timer wdgtimer;

void hw_wdg_feed(void)
{
    if(g_wdg_enable == 0) return;
    (WDI_STATUS == PIN_HIGH) ? (WDI_CLR) : (WDI_SET);
}

void hw_wgd_enable(rt_uint8_t _status)
{
    if(g_wdg_inited == 0) return;
    if(_status != 0) {
        rt_pin_mode(WDI_PIN, PIN_MODE_OUTPUT);
        hw_wdg_feed();
    }
    else {
        rt_pin_mode(WDI_PIN, PIN_MODE_INPUT);
    }
    g_wdg_enable = _status;
}

/**
 * 定时器超时回调
 * @param  : 
 * @return  : 
 */
static void wdgtimer_out(void *param)
{
    hw_wdg_feed();
}

/**
 * 看门狗初始化
 * @param  : 
 * @return :
 */
int hw_wdg_init(void)
{
    rt_timer_init(&wdgtimer, "wdgtimer",  \
    wdgtimer_out, RT_NULL, 500, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER );

	rt_timer_start(&wdgtimer);
	
    g_wdg_inited = 1;
    hw_wgd_enable(1);
    return 0;
}
#if APP_USING_HW_WDG
INIT_ENV_EXPORT(hw_wdg_init);
#endif 



