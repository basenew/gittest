/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : rt_fota_flags.c
 * Brief      : 

 * Change Logs
 * Date           Author          Notes
 * 2020-05-05     wuxiaofeng      first version        
 */

#include <rtthread.h>
#include "../../../ubt_common.h"
#include "rt_fota_cfg.h"
#include "rt_fota_flag.h"
#include "rt_fota.h"
#include "ee.h"


ALIGN(1)
static fota_flags_t fota_flags;

rt_err_t fota_flags_write(uint8_t *pBuffer, uint16_t NumToWrite)
{
    return ee_write(EE_ADDR_OTA_FLAGS, pBuffer, NumToWrite);
}

rt_err_t fota_flags_read(uint8_t *pBuffer, uint16_t NumToRead)
{
    return ee_read(EE_ADDR_OTA_FLAGS, pBuffer, NumToRead);
}

int fota_flags_init(void)
{
    if (fota_flags_read((uint8_t*)(&fota_flags), sizeof(fota_flags_t)) == RT_EOK)
    {
        if (fota_flags.initialized != FLAG_INIT) 
        {
            /*set flags default*/
            fota_flags.uflag.sflag.jump_to_where = FLAG_TO_APP;
            fota_flags.uflag.sflag.update_from = FLAG_FROM_DL;
            fota_flags.uflag.sflag.force_up = FLAG_NO;
            fota_flags.initialized = FLAG_INIT;
            fota_flags.errcode = RT_FOTA_NO_ERR;
            fota_flags.tobl_cnt = 0;
            rt_sprintf(fota_flags.app_version, "0.0.0");
            rt_sprintf(fota_flags.bl_version,  "0.0.0");
            rt_memset(fota_flags.reserved, 0x00, sizeof(fota_flags.reserved));
            fota_flags_write((uint8_t*)(&fota_flags), sizeof(fota_flags_t));
            rt_kprintf("fota flags initialized.\r\n");
        }
    }
    else {
        return RT_ERROR;
    }
    
    /*reflesh app_version to the RealVersion*/
    rt_sprintf(fota_flags.app_version, DF_APP_SOFTWARE_VERSION);
    fota_flags_write((uint8_t*)(&fota_flags), sizeof(fota_flags_t));
    
    return RT_EOK;
}
INIT_APP_EXPORT(fota_flags_init);

fota_flags_t* fota_flags_get(void)
{
    return &fota_flags;
}

