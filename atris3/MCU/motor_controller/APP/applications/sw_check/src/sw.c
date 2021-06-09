/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : sw.c
 * Brief      : simple module for sampling switch-units.

 * Change Logs
 * Date           Author          Notes
 * 2019-10-24     wuxiaofeng      first version        
 */
#include <rtthread.h>
#include <rthw.h>
#include "sw_def.h"
#include "app_cfg.h"

#define LOG_TAG              "pkg.sw"
#define LOG_LVL              LOG_LVL_DBG
#include <rtdbg.h>


#ifndef TASK_STACK_SIZE_SW
#define TASK_STACK_SIZE_SW 1024
#endif

#ifndef TASK_PRIORITY_SW
#define TASK_PRIORITY_SW 11
#endif

static uint8_t      g_table_size = 0;
static sw_unit_t*   g_table_head = RT_NULL;


#define  SW_LOCK()     level=rt_hw_interrupt_disable()
#define  SW_UNLOCK()   rt_hw_interrupt_enable(level)


static int32_t sw_get_index(sw_idx_t _idx)
{
    sw_unit_t*  ptable = RT_NULL;
    ptable = g_table_head;
    for (uint8_t index = 0; index < g_table_size; index ++) 
    {
        if(_idx == ptable->Idx) 
        {
            return index;
        }
        ptable ++;
    }
    return -1;
}


static void sw_units_init(void)
{
    sw_unit_t*  ptable = RT_NULL;
    ptable = g_table_head;
    for (uint8_t idx = 0; idx < g_table_size; idx ++)
    {
        ptable->Init(ptable);
        ptable ++;
    }
}

static void sw_process(void *_param)
{
    sw_unit_t *ptable = RT_NULL;
    ptable = g_table_head;
    while (1)
    {
        if (ptable->En != 0)
        {
            if (ptable->Tmr >= ptable->Ms / SW_POLL_SLICE)
            {
                ptable->Tmr = 0;
                ptable->CurStatus = ptable->Read(ptable);
                if ((ptable->CurStatus != ptable->LastStatus) && (++ptable->DbCnt >= ptable->Debonce / ptable->Ms))
                { //消抖
                    ptable->DbCnt = 0;
                    ptable->LastStatus = ptable->CurStatus;
                    if (ptable->CallBack != RT_NULL)
                    {
                        ptable->CallBack(ptable->Idx, ptable->CurStatus);
                    }
                }
            }
            else
            {
                ptable->Tmr ++;
            }
        }

        if (ptable == (g_table_head + g_table_size -1))
        {
            ptable = g_table_head;
        }
        else
        {
            ptable ++;  
        }

        rt_thread_mdelay(SW_POLL_SLICE);
    }
}

void sw_unit_attach(sw_idx_t idx, sw_callback_t cb)
{
    sw_unit_t*  ptable = RT_NULL;
    ptable = g_table_head;
    
    rt_base_t level;

    uint8_t index = sw_get_index(idx);
    if(cb != RT_NULL) 
    {
        SW_LOCK();
        (ptable+index)->CallBack = cb;
        SW_UNLOCK();
    }
}

void sw_unit_detach(sw_idx_t idx)
{
    sw_unit_t*  ptable = RT_NULL;
    ptable = g_table_head;

    rt_base_t level;

    uint8_t index = sw_get_index(idx);
    SW_LOCK();
    (ptable+index)->CallBack = RT_NULL;
    SW_UNLOCK();
}


void sw_unit_enable(sw_idx_t idx,  uint32_t ms)
{
    sw_unit_t*  ptable = RT_NULL;
    ptable = g_table_head;
    
    uint8_t index = sw_get_index(idx);
    rt_base_t level;
    SW_LOCK();
    (ptable+index)->En = 1;
    if(ms < SW_POLL_SLICE) {
        ms = SW_POLL_SLICE;
    }
    (ptable+index)->Ms = ms;
    (ptable+index)->Debonce = 2*ms;
    SW_UNLOCK();
}

void sw_unit_disable(sw_idx_t idx)
{
    sw_unit_t*  ptable = RT_NULL;
    ptable = g_table_head;
    uint8_t index = sw_get_index(idx);
    rt_base_t level;
    SW_LOCK();
    (ptable+index)->En = 0;
    (ptable+index)->Ms = SW_POLL_SLICE;
    (ptable+index)->Debonce = 2*SW_POLL_SLICE;
    SW_UNLOCK();
}

uint8_t sw_unit_read(sw_idx_t idx)
{
    sw_unit_t*  ptable = RT_NULL;
    ptable = g_table_head;
    uint8_t index = sw_get_index(idx);
    ptable += index;

    return ptable->Read(ptable);
}

int sw_init(void)
{
    g_table_size = sw_port_table_size();
    g_table_head = sw_port_table();
    if(g_table_size == 0 || g_table_head == RT_NULL) 
    {
        rt_kprintf("sw table is empty!\n");
        return -1;
    }
    
    sw_units_init();
    
    rt_thread_t thread = rt_thread_create("sw", \
                                  sw_process, \
                                  RT_NULL, \
                                  TASK_STACK_SIZE_SW, \
                                  TASK_PRIORITY_SW, \
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
INIT_APP_EXPORT(sw_init);





