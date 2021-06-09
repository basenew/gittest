/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : common.h.c
 * Brief      : public functions.

 * Change Logs
 * Date           Author          Notes
 * 2020-07-13     wuxiaofeng      first version        
 */

#include "common.h"


int32_t os_gettime_ms(void)
{
    return (rt_tick_get() * 1000 / RT_TICK_PER_SECOND);
}

