/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : rt_fota_flag.c
 * Brief      : 

 * Change Logs
 * Date           Author          Notes
 * 2020-05-05     wuxiaofeng      first version        
 */

#include <rtthread.h>
#include "rt_fota_cfg.h"
#include "rt_fota_flag.h"
#include "rt_fota.h"
#include "at24cxx.h"
#include "../../ubt_common.h"


#define EE_ADDR 0x00
#define FLAGS_POS_OFFSET 0


ALIGN(1)
static fota_flags_t fota_flags;
static at24cxx_device_t ee_dev = RT_NULL;

rt_err_t ee_init(void)
{
    ee_dev = at24cxx_init("i2c2", EE_ADDR);
    return (ee_dev == RT_NULL) ?  RT_ERROR : RT_EOK;
}

rt_err_t ee_write(uint8_t *pBuffer, uint16_t NumToWrite)
{
    return at24cxx_write(ee_dev, FLAGS_POS_OFFSET, pBuffer, NumToWrite);
}

rt_err_t ee_read(uint8_t *pBuffer, uint16_t NumToRead)
{
    return at24cxx_read(ee_dev, FLAGS_POS_OFFSET, pBuffer, NumToRead);
}

int fota_flags_init(void)
{
    if (ee_init() == RT_EOK)
    {
        ee_read((uint8_t*)(&fota_flags), sizeof(fota_flags_t));
        if (fota_flags.initialized != FLAG_INIT) 
        {
            /*set flags default*/
            fota_flags.uflag.sflag.jump_to_where = FLAG_TO_BL;
            fota_flags.uflag.sflag.update_from = FLAG_FROM_DL;
            fota_flags.uflag.sflag.force_up = FLAG_NO;
            fota_flags.initialized = FLAG_INIT;
            fota_flags.errcode = RT_FOTA_NO_ERR;
            fota_flags.tobl_cnt = 0;
            rt_sprintf(fota_flags.app_version, "0.0.0");
            rt_sprintf(fota_flags.bl_version,  "0.0.0");
            rt_memset(fota_flags.reserved, 0x00, sizeof(fota_flags.reserved));
            ee_write((uint8_t*)(&fota_flags), sizeof(fota_flags_t));
            rt_kprintf("fota flags initialized.\r\n");
        }
    }
    else 
    {
        return RT_ERROR;
    }

    rt_sprintf(fota_flags.bl_version,  DF_BL_SOFTWARE_VERSION);
    ee_write((uint8_t*)(&fota_flags), sizeof(fota_flags_t));
    return RT_EOK;
}
INIT_APP_EXPORT(fota_flags_init);

fota_flags_t* fota_flags_get(void)
{
    return &fota_flags;
}

