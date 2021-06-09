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
#include <rtdevice.h>
#include "../../ubt_common.h"
#include <board.h>
#include "app_cfg.h"
#include "msg_canpkg.h"
#include "led.h"
#include "hw_wdg.h"
#include "rt_fota.h"
#include "ota.h"

#define LOG_TAG              "main"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>



#include "led_1.h"
#include "adc_sample.h"
#include "filter.h"
#include "pid.h"
#include "pi.h"
#include "motor_driver.h"
#include "time_update.h"
#include "encoder_driver.h"
#include "hall_driver.h"

#include "electric_angle.h"


#include "motor_protect.h"



extern volatile uint8_t motor_sw;
void delay_about_ms(uint16_t ms)
{
	uint16_t i,j;
	for(i=0; i<ms; i++)
		for(j=0; j<15000; j++);
}
void motor(void)
{
	led_init();
	red_led_er_init();
	LED_RED_ER2_ON; 
	delay_about_ms(200);
	LED_RED_ER2_OFF;
	//-----pid电流采样
	adc_current_init();
	CAL_HIGHT;
	delay_about_ms(200);
	CAL_LOW;
	//-----电机桥臂控制初始化
	motor2_gpio_init();
	motor2_tim_pwm_init();
	motor1_gpio_init();
	motor1_tim_pwm_init();
	//-----霍尔传感器初始化
	motor_hall_interupt_init();
	motor1_hall_init();
	motor2_hall_init();
	//-----滑动滤波器初始化
	Window_filter_Init();
	//-----时间计时器初始化
//	pid_time_init();
	//--------编码器初始化
	motor1_encoder_init();
	motor2_encoder_init();
	//--------电机启动 	
	get_motor1_hall();
	get_motor2_hall();
	hall_electric_angle_4000();
	hall2_electric_angle_4000();
	set_votage_offset();
	Window_filter_Update_all();
	//-----------PID时间定时器
	foc_time_init();
    //pi_set_motor_speed(0.5, 0.5);
    GPIO_SetBits(MOTOR1_ENABLE_PORT, MOTOR1_ENABLE_PIN);
	GPIO_SetBits(MOTOR2_ENABLE_PORT, MOTOR2_ENABLE_PIN);
	TIM_CtrlPWMOutputs(MOTOR1_TIM, ENABLE);
	TIM_CtrlPWMOutputs(MOTOR2_TIM, ENABLE);
	//-----pid位置环初始化
	pid_param_init_all();
	pi_init_motor1();
	pi_init_motor2();
     pi_set_motor_speed(5, 5);   
	rt_kprintf("motor init finish!\r\n");
	rt_kprintf("motor init finish!\r\n");   
}

static void app_version_print(void)
{   
    fota_flags_t* pflags = fota_flags_get();

    rt_kprintf("\r\n");
    rt_kprintf("\r\n");
    rt_kprintf("------------------------------------------------------\r\n");
    rt_kprintf("             APP BY UBTECH            \r\n");
    rt_kprintf("   Version: %d  |  %s  \r\n", hardware_version_get(), pflags->app_version);
    rt_kprintf("   Build  : %s  |  %s  \r\n", __DATE__, __TIME__);
    rt_kprintf("------------------------------------------------------\r\n");
    rt_kprintf("\r\n");
}

static int app_env_init(void)
{
    canpkg_publisher_init();
    canpkg_subscriber_init();
    return 0;
}    

static int app_modules_init(void)
{
    ota_init();
    return 0;
}    

int main(void)
{
    /*ensure threads initialized before main start up*/
    rt_thread_mdelay(500);
    
    app_version_print();

    app_env_init();
    
    app_modules_init();
    motor();

    // rt_thread_mdelay(5000);
    // extern void http_ota(rt_uint8_t argc, char **argv);
    // static char* test_cmd[1][4] = {
    //     [0] = {"http_ota", "12", "download", "app"},
    // }; 
    // http_ota(4, test_cmd[0]);
    
//    uint32_t cnt = 0;
//    while (1)
//    {
//        LOG_W("%d->01234567890123456789012345678901234567890123456789012345678901234567890123456789", cnt++);
//        rt_thread_mdelay(20);
//    }

//    return RT_EOK;
}




