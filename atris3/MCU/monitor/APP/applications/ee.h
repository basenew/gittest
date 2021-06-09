/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : ee.h
 * Brief      : kernal board eeprom oporation

 * Change Logs
 * Date           Author          Notes
 * 2020-05-05     wuxiaofeng      first version        
 */

#ifndef __EE_H__
#define __EE_H__


#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    EE_ADDR_OTA_FLAGS = 0u,
    //..
} ee_addr_t;


rt_err_t ee_write(ee_addr_t idx, uint8_t *pBuffer, uint16_t NumToWrite);
rt_err_t ee_read(ee_addr_t idx, uint8_t *pBuffer, uint16_t NumToWrite);

/*raw api*/
rt_err_t ee_raw_write(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);
rt_err_t ee_raw_read(uint32_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead);

#ifdef __cplusplus
}
#endif

#endif


