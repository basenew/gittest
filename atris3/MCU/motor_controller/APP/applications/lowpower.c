/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : lowpower.c
 * Brief      : lowpower manager

 * Change Logs
 * Date           Author        Version       Notes
 * 2020-07-03     wuxiaofeng    v1.0               
 */

#include "board.h"
#include "lowpower.h"


static void enter_lowpower_prepare(void)
{
    //__HAL_RCC_APB2_FORCE_RESET();
    //__HAL_RCC_GPIOC_CLK_DISABLE();
    //__HAL_RCC_GPIOD_CLK_DISABLE();
    //__HAL_RCC_GPIOA_CLK_DISABLE();
    //__HAL_RCC_GPIOB_CLK_DISABLE();
}


void enter_standby_mode(void)
{  
    /* prepare something*/
    enter_lowpower_prepare();
    
    /* Enable Power Clock*/
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Allow access to Backup */
    HAL_PWR_EnableBkUpAccess();
    
    /* Reset RTC Domain */
    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();

    /* Disable all used wakeup sources: Pin1(PA.0) */
    //HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
    /* Re-enable all used wakeup sources: Pin1(PA.0) */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    
    /* Clear all related wakeup flags */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    
    /* Request to enter STANDBY mode  */
    HAL_PWR_EnterSTANDBYMode(); 
    
    /* when wakeup, reset the mcu*/
    rt_hw_cpu_reset();
}









