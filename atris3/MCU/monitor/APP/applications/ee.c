/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : ee.c
 * Brief      : kernal board eeprom oporation

 * Change Logs
 * Date           Author          Notes
 * 2020-05-05     wuxiaofeng      first version        
 */
#include "rtthread.h"
#include "at24cxx.h"
#include "ee.h"


#define EE_ADDR 0x00


typedef struct
{
    ee_addr_t idx;
    int addr;
    int size;
} ee_map_t;

/*eeprom address allocation table */
static ee_map_t ee_map[] = 
{
    { EE_ADDR_OTA_FLAGS, 0x0,  16*4 },
    
};


static at24cxx_device_t ee_dev = RT_NULL;

rt_err_t ee_raw_write(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{
    if (ee_dev == RT_NULL || pBuffer == RT_NULL) {
        return RT_ERROR;
    }
    return at24cxx_write(ee_dev, WriteAddr, pBuffer, NumToWrite);
}

rt_err_t ee_raw_read(uint32_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead)
{
    if (ee_dev == RT_NULL || pBuffer == RT_NULL) {
        return RT_ERROR;
    }
    return at24cxx_read(ee_dev, ReadAddr, pBuffer, NumToRead);
}

rt_err_t ee_write(ee_addr_t idx, uint8_t *pBuffer, uint16_t NumToWrite)
{
    if (NumToWrite > ee_map[idx].size) {
        NumToWrite = ee_map[idx].size;
    }
    uint32_t addr = ee_map[idx].addr;
    return ee_raw_write(addr, pBuffer, NumToWrite);
}

rt_err_t ee_read(ee_addr_t idx, uint8_t *pBuffer, uint16_t NumToWrite)
{
    if (NumToWrite > ee_map[idx].size) {
        NumToWrite = ee_map[idx].size;
    }
    uint32_t addr = ee_map[idx].addr;
    return ee_raw_read(addr, pBuffer, NumToWrite);
}

// TODO  page w r

int ee_init(void)
{
    if (ee_dev == RT_NULL) {
        ee_dev = at24cxx_init("i2c2", EE_ADDR);
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(ee_init);
