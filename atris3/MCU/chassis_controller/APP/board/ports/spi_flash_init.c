/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-27     SummerGift   add spi flash port file
 */

#include <rtthread.h>
#include <dfs_posix.h>
#include "spi_flash.h"
#include "spi_flash_sfud.h"
#include "drv_spi.h"
#include "fal.h"
#include "dfs_fs.h"
#include <easyflash.h>
#include "spi_flash_init.h"


#if defined(BSP_USING_SPI_FLASH)

#define FS_PARTITION_NAME "fs"


static uint8_t s_flash_init_ok = 0;

static rt_err_t rt_hw_spi_flash_init(void)
{
    static uint8_t s_init_ok = 0;
    
    if (s_init_ok != 0) {
        return RT_EOK;
    }
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    rt_hw_spi_device_attach("spi1", "spi10", GPIOA, GPIO_PIN_4);
    
    if (RT_NULL == rt_sfud_flash_probe(FAL_USING_NOR_FLASH_DEV_NAME, "spi10"))
    {
        return -RT_ERROR;
    }
    
    s_init_ok = 1;
    return RT_EOK;
}

static rt_err_t rt_fal_part_init(void)
{
    if (fal_init() >= 0)
        return RT_EOK;
    else 
        return -RT_ERROR;
}

static rt_err_t rt_hw_easyflash_init(void)
{
    if (easyflash_init() == EF_NO_ERR)
        return RT_EOK;
    else
        return -RT_ERROR;
}


static rt_err_t fs_mount(void)
{
    struct rt_device *mtd_dev = RT_NULL;
    rt_err_t ret = -RT_ERROR;
    
    mtd_dev = fal_mtd_nor_device_create(FS_PARTITION_NAME);
    if (!mtd_dev) {
        rt_kprintf("Can't create a mtd device on '%s' partition.\n", FS_PARTITION_NAME);
    }
    else
    {
        if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
        {
#if defined(BSP_USING_SDCARD)
            //mount sdcard to /sdcard
            mkdir("/sdcard", 0x777);
#endif
            ret = RT_EOK;
        }
        else
        {
            dfs_mkfs("lfs", FS_PARTITION_NAME);
            if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
            {
                ret = RT_EOK;
            }
            else {
                rt_kprintf("mount '%s' partition failed.\n", FS_PARTITION_NAME);
            }
        }
    }
    return ret;
}


rt_err_t spi_flash_init(void)
{
    s_flash_init_ok = E_SPI_FLASH_ERR;
    
    if (rt_hw_spi_flash_init() != RT_EOK) {
        return -RT_ERROR;
    }
    s_flash_init_ok = E_SPI_FLASH_PROBE_OK;
    
    if (rt_fal_part_init() != RT_EOK) {
        return -RT_ERROR;
    }
    s_flash_init_ok = E_SPI_FLASH_FAL_OK;
    
    if (rt_hw_easyflash_init() != RT_EOK) {
        return -RT_ERROR;
    }
    s_flash_init_ok = E_SPI_FLASH_EF_OK;

    if (fs_mount() != RT_EOK) {
        return -RT_ERROR;
    }
    s_flash_init_ok = E_SPI_FLASH_FS_MOUNT_OK;
    
    return RT_EOK;
}

uint8_t spi_flash_check_initial(void)
{
    return s_flash_init_ok;
}


#endif







