/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : main.c
 * Brief      : 

 * Change Logs
 * Date           Author          Notes
 * 2020-05-05     wuxiaofeng      first version        
 */


#include <rtthread.h>
#include "rt_fota_cfg.h"
#include "rt_fota_flag.h"

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>
#include <shell.h>
#endif

#define DBG_ENABLE
#define DBG_SECTION_NAME                    "main"

#ifdef RT_MAIN_DEBUG
#define DBG_LEVEL                           DBG_LOG
#else
#define DBG_LEVEL                           DBG_INFO
#endif

#define DBG_COLOR
#include <rtdbg.h>

void rt_fota_print(void)
{   
    fota_flags_t* pflags = fota_flags_get();
    
    LOG_RAW("\r\n");
    LOG_RAW("------------------------------------------------------\r\n");
    LOG_RAW("           BOOTLOADER BY UBTECH            \r\n");
    LOG_RAW("     Version: %s   |  Build: %s     \r\n", pflags->bl_version, __DATE__);
    LOG_RAW("------------------------------------------------------\r\n");
}

int main(void)
{    
#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
	finsh_set_prompt("rt-fota />");
#endif
    
    rt_fota_print();
    
    extern void rt_fota_init(void);
    rt_fota_init();

    return RT_EOK;
}

