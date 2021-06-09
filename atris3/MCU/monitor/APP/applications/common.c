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

/**
 * ??uint32_t??????????????bit-1
 * @param   _param : uint32_t ??
 * @param   _n     : ?????
 * @return  0~31   : ??? bit-1 ?? 
 			>=_n   : ????
 */
uint8_t find_first_1bit_uint32(uint32_t _param, uint8_t _n)
{
    int8_t i = 0;
    
    if (_n >= 32) _n = 32;
    
    for (i = 0; i < _n; i++)
    {
        if (_param & (1 << i))
            break;
    }
    return i;
}


