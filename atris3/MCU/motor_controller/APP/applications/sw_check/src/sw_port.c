/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : sw_port.c
 * Brief      : simple module for sampling switch-units.

 * Change Logs
 * Date           Author          Notes
 * 2019-10-24     wuxiaofeng      first version        
 */
#include "rtthread.h"
#include "rtdevice.h"
#include "sw_def.h"


static int32_t io_init(sw_unit_t* unit);
static uint8_t io_read(sw_unit_t* unit);

static sw_unit_t swtable[] = 
{
{SW_SOFT_BUTTUN,  0 , SW_POLL_SLICE, 2*SW_POLL_SLICE, PIN_HIGH, PIN_HIGH, 0, io_init, io_read, RT_NULL,},   // PA0

};


static int32_t io_init(sw_unit_t* unit)
{
    rt_pin_mode(unit->Pin, PIN_MODE_INPUT);
    return 0;
}

static uint8_t io_read(sw_unit_t* unit)
{
    return rt_pin_read(unit->Pin);
}


/*
static int32_t bus_init(sw_unit_t* unit)
{

}
static int32_t bus_read(sw_unit_t* unit)
{

}
*/



sw_unit_t* sw_port_table(void)
{
    return &swtable[0];
}


uint8_t sw_port_table_size(void)
{
    return (sizeof(swtable)/sizeof(sw_unit_t));
}



