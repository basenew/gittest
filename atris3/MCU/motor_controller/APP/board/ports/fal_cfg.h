/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-5      SummerGift   first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtthread.h>
#include <board.h>
#include "../../../ubt_common.h"

#define FLASH_SIZE_GRANULARITY_16K   (4 * 16 * 1024)
#define FLASH_SIZE_GRANULARITY_64K   (64 * 1024)
#define FLASH_SIZE_GRANULARITY_128K  (7 * 128 * 1024)

#define STM32_FLASH_START_ADRESS_16K  STM32_FLASH_START_ADRESS
#define STM32_FLASH_START_ADRESS_64K  (STM32_FLASH_START_ADRESS_16K + FLASH_SIZE_GRANULARITY_16K)
#define STM32_FLASH_START_ADRESS_128K (STM32_FLASH_START_ADRESS_64K + FLASH_SIZE_GRANULARITY_64K)


#define JUMPER_ADDR (DF_JUMPER_LINKER_ADDR - 0x08000000)
#define JUMPER_SIZE (DF_BOOTLOADER_LINKER_ADDR - DF_JUMPER_LINKER_ADDR)
#define BOOTLOADER_ADDR (JUMPER_ADDR + JUMPER_SIZE)
#define BOOTLOADER_SIZE (DF_APP_LINKER_ADDR - DF_BOOTLOADER_LINKER_ADDR)
#define APP_ADDR (BOOTLOADER_ADDR + BOOTLOADER_SIZE)
#define APP_SIZE (0x08100000 - DF_APP_LINKER_ADDR)


extern const struct fal_flash_dev stm32f4_onchip_flash;
extern struct fal_flash_dev nor_flash0;

/* flash device table */
#define FAL_FLASH_DEV_TABLE     \
{                               \
    &stm32f4_onchip_flash,      \
    &nor_flash0,                \
}
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG

/* partition table */
#define FAL_PART_TABLE \
{  \
    {FAL_PART_MAGIC_WROD,      "jumper",    "onchip_flash",                    JUMPER_ADDR,       JUMPER_SIZE, 0}, \
    {FAL_PART_MAGIC_WROD,          "bl",    "onchip_flash",                BOOTLOADER_ADDR,   BOOTLOADER_SIZE, 0}, \
    {FAL_PART_MAGIC_WROD,         "app",    "onchip_flash",                       APP_ADDR,          APP_SIZE, 0}, \
    {FAL_PART_MAGIC_WROD,    "download",    FAL_USING_NOR_FLASH_DEV_NAME,                0,  (1*1024) * 1024, 0},  \
    {FAL_PART_MAGIC_WROD,      "backup",    FAL_USING_NOR_FLASH_DEV_NAME,  (1*1024) * 1024,  (1*1024) * 1024, 0},  \
    {FAL_PART_MAGIC_WROD,       "idle0",    FAL_USING_NOR_FLASH_DEV_NAME,  (2*1024) * 1024,  (1*1024) * 1024, 0},  \
    {FAL_PART_MAGIC_WROD,          "kv",    FAL_USING_NOR_FLASH_DEV_NAME,  (3*1024) * 1024,  (1*1024) * 1024, 0},  \
    {FAL_PART_MAGIC_WROD,          "fs",    FAL_USING_NOR_FLASH_DEV_NAME,  (4*1024) * 1024,  (10*1024) * 1024, 0}, \
    {FAL_PART_MAGIC_WROD,       "idle1",    FAL_USING_NOR_FLASH_DEV_NAME, (14*1024) * 1024,   (2*1024) * 1024, 0}, \
}
#endif /* FAL_PART_HAS_TABLE_CFG */
#endif /* _FAL_CFG_H_ */
