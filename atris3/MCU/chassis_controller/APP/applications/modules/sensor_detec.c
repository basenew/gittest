/*
 * COPYRIGHT (C) Copyright 2012-2017; UBT TECH; SHENZHEN, CHINA
 *
 * File       : sensor_adc.c
 * Brief      : ADC类传感器数据采集

 * Change Logs
 * Date           Author        Version       Notes
 * 2018-12-18     wuxiaofeng    v1.0
 *
 *
 */
#include "math.h"
#include "common.h"
#include "app_cfg.h"
//#include "fan.h"
#include "sensor_detec.h"
#include "sensor.h"
#include "ntc_sample.h"
#include "curr_sample.h"

#define LOG_TAG              "sensor"
#define LOG_LVL               LOG_LVL_INFO
#include <ulog.h>

//--------------------------------------------------------------------------------------------------

//传感器真实值
//#define  NTC_SENSOR_NUM   2
//static int8_t   g_ntc_temp[NTC_SENSOR_NUM] = {0};
static int32_t g_hsu_temp     = 0;
static uint16_t g_hsu_humi     = 0;

//传感器信息打印相关
static uint8_t  g_sensor_info_print_flag    = NO;
static uint32_t g_sensor_info_time_interval = 1000;
//---------------------------------------------------------------------------------------------------------------------------------

static rt_device_t temp_hsu_dev;
static rt_device_t humi_hsu_dev;

//上传can的数据偏移值
#define SENSOR_HSU_TEMP_REPORT_OFFSET  300
#define SENSOR_HSU_HUMI_REPORT_OFFSET  0

static int32_t hsu_find(void)
{
    //find hsu_chm_01a sensor
    temp_hsu_dev = rt_device_find("temp_hsu");
    humi_hsu_dev = rt_device_find("humi_hsu");


    if (temp_hsu_dev == RT_NULL || humi_hsu_dev == RT_NULL)
    {
        LOG_E("cannot find hsu sensor!");
        return -1;
    }

    if (rt_device_open(temp_hsu_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("open temp_hsu_dev failed!");
        return -1;
    }
    if (rt_device_open(humi_hsu_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("open humi_hsu_dev failed!");
        return -1;
    }
    return 0;
}

static void hsu_sensor_read(void)
{
    struct rt_sensor_data data_temp;
    struct rt_sensor_data data_humi;
//    uint8_t txbuf[8] = {0};

    if (temp_hsu_dev == RT_NULL || humi_hsu_dev == RT_NULL)
    {
        data_temp.data.temp = 0xFFFF;
        data_humi.data.humi = 0xFFFF;
    }
    else
    {
        rt_memset(&data_temp, 0, sizeof(struct rt_sensor_data));
        rt_memset(&data_humi, 0, sizeof(struct rt_sensor_data));

        if (rt_device_read(temp_hsu_dev, 0, &data_temp, 1) != 1) {
            data_temp.data.temp = 0xFFFF;
        }
        if (rt_device_read(humi_hsu_dev, 0, &data_humi, 1) != 1) {
            data_humi.data.humi = 0xFFFF;
        }
    }

    //only for being readed.
    g_hsu_temp = data_temp.data.temp;
    g_hsu_humi = data_humi.data.humi;

    //add offset for reporting.
//    data_temp.data.temp += SENSOR_HSU_TEMP_REPORT_OFFSET;
//    data_humi.data.humi += SENSOR_HSU_HUMI_REPORT_OFFSET;
	
//    rt_kprintf("BOARD : temp: %d.%d \t humi: %d%%\n", g_hsu_temp/10, g_hsu_temp%10, g_hsu_humi/10);
}

static void sensor_info_print(void)
{
    static uint32_t s_print_timer = 0;
	int8_t g_ntc_temp[4] = {0};
	

    if (g_sensor_info_print_flag != NO)
    {
        if (os_gettime_ms() - s_print_timer >= g_sensor_info_time_interval)
        {
            s_print_timer = os_gettime_ms();
			ntc_get_temp_data(g_ntc_temp);
            rt_kprintf("\n");
            rt_kprintf("NTC   : tmp1: %d \t tmp2: %d \t tmp3: %d \t tmp4: %d\n", g_ntc_temp[0], g_ntc_temp[1], g_ntc_temp[2], g_ntc_temp[3]);
            rt_kprintf("BOARD : temp: %d.%d \t humi: %d%%\n", g_hsu_temp/10, g_hsu_temp%10, g_hsu_humi/10);
        } else if (os_gettime_ms() < s_print_timer) {
            s_print_timer = os_gettime_ms();
        }
    }
}

/**
 * 传感器采集任务
 * @param  _param  : 任务参数
 * @return         : void
 */
static void sensor_detec_task_main(void* _param)
{
    uint32_t sensor_timer_10ms = 0;
    uint32_t sensor_timer_20ms = 0;
    uint32_t sensor_timer_1000ms = 0;

    while (1)
    {
        if (os_gettime_ms() - sensor_timer_10ms >= 10)
        {
            sensor_timer_10ms = os_gettime_ms();
            ntc_sample();
        } else if (os_gettime_ms() < sensor_timer_10ms) {
            sensor_timer_10ms = os_gettime_ms();
        }
		
        if (os_gettime_ms() - sensor_timer_20ms >= 20)
        {
            sensor_timer_20ms = os_gettime_ms();
            curr_read();
        } else if (os_gettime_ms() < sensor_timer_20ms) {
            sensor_timer_20ms = os_gettime_ms();
        }
		
        if (os_gettime_ms() - sensor_timer_1000ms >= 1000)
        {
            sensor_timer_1000ms = os_gettime_ms();
            hsu_sensor_read();
        } else if (os_gettime_ms() < sensor_timer_1000ms) {
            sensor_timer_1000ms = os_gettime_ms();
        }

        sensor_info_print();
		curr_print();
        
		rt_thread_mdelay(10);
    }
}

int32_t sensor_detec_init(void)
{
	ntc_init();

    hsu_find();
	
	curr_sample_init();
if(DF_THREAD_STATIC_MEMORY == 0){
    rt_thread_t snsr_thread = rt_thread_create("task_sensor", \
                              sensor_detec_task_main, \
                              RT_NULL, \
                              TASK_STACK_SIZE_SENSOR, \
                              TASK_PRIORITY_SENSOR_ADC, \
                              20);
    if (snsr_thread != RT_NULL)
    {
        rt_thread_startup(snsr_thread);
    }
    else
    {
        return -1;
    }
}else{
    static struct rt_thread sensor_thread;
    ALIGN(RT_ALIGN_SIZE)
    static char sensor_thread_stack[TASK_STACK_SIZE_SENSOR]; 
    	rt_err_t result = RT_EOK;
    result = rt_thread_init(&sensor_thread,
                            "task_sensor",
                            sensor_detec_task_main, RT_NULL,
                            &sensor_thread_stack[0], sizeof(sensor_thread_stack),
                            TASK_PRIORITY_SENSOR_ADC, 20);

    if (result == RT_EOK){
    	rt_thread_startup(&sensor_thread);
        return 0;
    }else {
    
        LOG_I("%s thread create failed.",__FUNCTION__);
        return -1;
    }
    	
    
}    
    
    
    
    return 0;
}

int32_t sensor_get_hsu_temp(void)
{
	return g_hsu_temp;
}

uint16_t sensor_get_hsu_hum(void)
{
	return g_hsu_humi;
}
//-------------------------------------------------------------------------------------------------------------------------

#include "finsh.h"

static void sensor_info(uint8_t argc, char **argv)
{
    if (argc < 2 || argc > 3)
    {
        rt_kprintf("Please input: sensor_info <1/0> <ms, default 1000>\n");
    }
    else
    {
        g_sensor_info_print_flag = atoi(argv[1]);

        if (argc == 3)
        {
            uint32_t _ms = atoi(argv[2]);
            if (_ms < 10) _ms = 10;
            g_sensor_info_time_interval = _ms;
        }
    }
}
MSH_CMD_EXPORT(sensor_info, print sensor information);

