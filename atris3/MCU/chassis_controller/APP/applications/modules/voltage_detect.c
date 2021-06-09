/*
 * COPYRIGHT (C) Copyright 2018-2028; UBT TECH; SHENZHEN, CHINA
 *
 * File       : remote_ctrl.c
 * Brief      : 遥控器功能支持

 * Change Logs
 * Date           Author        Version       Notes
 * XXX           licaixia       v1.0
 * 20190331      wuxiaofeng     v1.1          RTT
 */

#include "common.h"
#include "app_cfg.h"
#include "voltage_detect.h"
#include "rtdevice.h"
#include "drv_spi.h"

#define LOG_TAG              "remote"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>

#define VOL_DET_DEVICE_NAME		"spi20"


uint8_t rst[2] = {0x00,0x40};   //reset
uint8_t cmd[16][2] = {{0x10,0x80},  //ch1 
					  {0x11,0x00},
					  {0x11,0x80},
					  {0x12,0x00},
					  {0x12,0x80},
					  {0x13,0x00},
					  {0x13,0x80},
					  {0x14,0x00},
					  {0x14,0x80},
					  {0x15,0x00},
					  {0x15,0x80},
					  {0x16,0x00},
					  {0x16,0x80},
					  {0x17,0x00},
					  {0x17,0x80},
					  {0x10,0x00}};
uint16_t vol_adc_value[16];		//电压ADC值
uint16_t vol_det_value[16];		//检测位置电压值，单位mv
uint16_t vol_converted_value[16];	//转换后的对应检测电压值，单位mv
static uint8_t g_voltage_adc_print_flag = NO;
static uint32_t g_voltage_adc_time_interval = 1000;
static uint8_t g_voltage_voldata_print_flag = NO;
static uint32_t g_voltage_voldata_time_interval = 1000;
					  
/* 打印电压检测ADC原始数据 */
static void voltage_adc_print(void)
{
    static uint32_t s_print_timer = 0;

    if (g_voltage_adc_print_flag != NO)
    {
        if (os_gettime_ms() - s_print_timer >= g_voltage_adc_time_interval)
        {
            s_print_timer = os_gettime_ms();
			
			rt_kprintf("vol adc:ch0-%d  ch1-%d  ch2-%d  ch3-%d  ch4-%d  ch5-%d\n", 
						vol_adc_value[0], vol_adc_value[1], vol_adc_value[2], vol_adc_value[3], vol_adc_value[4], vol_adc_value[5]);
        } else if (os_gettime_ms() < s_print_timer) {
            s_print_timer = os_gettime_ms();
        }
    }
}
/* 打印电压检测电压值 */
static void voltage_voldata_print(void)
{
    static uint32_t s_print_timer = 0;

    if (g_voltage_voldata_print_flag != NO)
    {
        if (os_gettime_ms() - s_print_timer >= g_voltage_voldata_time_interval)
        {
            s_print_timer = os_gettime_ms();
			
			rt_kprintf("vol_val:ch0-%d  ch1-%d  ch2-%d  ch3-%d  ch4-%d  ch5-%d\n", 
						vol_converted_value[0], vol_converted_value[1], vol_converted_value[2], vol_converted_value[3], vol_converted_value[4], vol_converted_value[5]);
        } else if (os_gettime_ms() < s_print_timer) {
            s_print_timer = os_gettime_ms();
        }
    }
}

static void voltage_convert(uint16_t vol_data, uint8_t _ch)
{
	switch(_ch)
	{
		case 0:
			vol_converted_value[_ch] = vol_data * 215/15;
			break;
		
		case 1:
			vol_converted_value[_ch] = vol_data * 215/15;
			break;
		
		case 2:
			vol_converted_value[_ch] = vol_data * 215/15;
			break;
		
		case 3:
			vol_converted_value[_ch] = vol_data * 12/2;
			break;
		
		case 4:
			vol_converted_value[_ch] = vol_data * 12/2;
			break;
		
		case 5:
			vol_converted_value[_ch] = vol_data * 80/33;
			break;
		
		default :
			break;
	}
}

static void voltage_detect_main(void* _param)
{
	struct rt_spi_device *spi_dev_vol_det;
	uint8_t rx_buf[2];
	uint8_t i;
	uint8_t ch;
	
	spi_dev_vol_det = (struct rt_spi_device *)(rt_device_find(VOL_DET_DEVICE_NAME));
    if (!spi_dev_vol_det)
    {
        rt_kprintf("spi sample run failed! can't find spi2!\n");
    }
    else
    {
        /* config spi */
        {
            struct rt_spi_configuration cfg;
            cfg.data_width = 8;
            cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
            cfg.max_hz = 10 * 1000 * 1000; /* 10M */
            rt_spi_configure(spi_dev_vol_det, &cfg);
        }
	}
	rt_thread_mdelay(20);
//	rt_spi_transfer(spi_dev_vol_det, rst, rx_buf, 2);
    while(1)
    {   
		for(i=0; i<16; i++)
		{
			rt_thread_mdelay(10);
			
			rt_spi_transfer(spi_dev_vol_det, &cmd[i][0], rx_buf, 2);
			ch = (uint8_t)(rx_buf[0] >> 4);
			vol_adc_value[ch] = BYTETOSHORT((rx_buf[0]&0x0f),rx_buf[1]);
//			rt_kprintf("ch %d: 0x%02X\t0x%02X\n", i, rx_buf[0], rx_buf[1]);
			vol_det_value[ch] = (vol_adc_value[ch]*2500/4096);
			voltage_convert(vol_det_value[ch], ch);
		}
		voltage_adc_print();
		voltage_voldata_print();
		
		rt_thread_mdelay(100);

    }
}


/**
 * 模块初始化
 * @param      : void
 * @return     : void
 */
int32_t voltage_detect_init(void)
{  

if(DF_THREAD_STATIC_MEMORY == 0){    
   rt_thread_t thread = rt_thread_create("task_voltage_detect", \
                                         voltage_detect_main, \
                                         RT_NULL, \
                                         TASK_STACK_SIZE_VOLTAGEDET, \
                                         TASK_PRIORITY_VOLTAGEDET, \
                                         20);
   if (thread != RT_NULL)
   {
       rt_thread_startup(thread);
   }
   else
   {
       return -1;
   }
}else{
    static struct rt_thread voltage_detect_thread;
    ALIGN(RT_ALIGN_SIZE)
    static char voltage_detect_thread_stack[TASK_STACK_SIZE_VOLTAGEDET]; 
    	rt_err_t result = RT_EOK;
    result = rt_thread_init(&voltage_detect_thread,
                            "voltage_detect",
                            voltage_detect_main, RT_NULL,
                            &voltage_detect_thread_stack[0], sizeof(voltage_detect_thread_stack),
                            TASK_PRIORITY_VOLTAGEDET, 20);

    if (result == RT_EOK){
    	rt_thread_startup(&voltage_detect_thread);
        return 0;
    }else {
    
        LOG_I("%s thread create failed.",__FUNCTION__);
        return -1;
    }
    	
    
}        
    return 0;
}


uint16_t vol_get_data(uint8_t ch)
{
	return vol_converted_value[ch];
}

uint16_t get_voltage(uint8_t ch)
{
	return vol_converted_value[ch];
}

static int rt_hw_spi_voltage_det_init(void)
{
    __HAL_RCC_GPIOI_CLK_ENABLE();

    return rt_hw_spi_device_attach("spi2", VOL_DET_DEVICE_NAME, GPIOI, GPIO_PIN_0);;
}
INIT_DEVICE_EXPORT(rt_hw_spi_voltage_det_init);
//---------------------------------------------------------------------------------

#include "finsh.h"

static void vol_adc(uint8_t argc, char **argv)
{
    if (argc < 2 || argc > 3)
    {
        rt_kprintf("Please input: vol_adc <1/0> <ms, default 1000>\n");
    }
    else
    {
        g_voltage_adc_print_flag = atoi(argv[1]);

        if (argc == 3)
        {
            uint32_t _ms = atoi(argv[2]);
            if (_ms < 10) _ms = 10;
            g_voltage_adc_time_interval = _ms;
        }
    }
}
MSH_CMD_EXPORT(vol_adc, print voltage adc information);

static void vol_data(uint8_t argc, char **argv)
{
    if (argc < 2 || argc > 3)
    {
        rt_kprintf("Please input: vol_data <1/0> <ms, default 1000>\n");
    }
    else
    {
        g_voltage_voldata_print_flag = atoi(argv[1]);

        if (argc == 3)
        {
            uint32_t _ms = atoi(argv[2]);
            if (_ms < 10) _ms = 10;
            g_voltage_voldata_time_interval = _ms;
        }
    }
}
MSH_CMD_EXPORT(vol_data, print voltage data information);











