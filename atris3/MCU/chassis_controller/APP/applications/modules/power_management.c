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
#include "power_management.h"
#include "voltage_detect.h"
#include "battery.h"
#include "bms.h"
#include "power_ctrl.h"

#define LOG_TAG              "charger"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>


static uint8_t power_led_state = 0;
static uint8_t power_key_value = 0;
static charger_data_t charger_data = {0};
static udock_t g_udock = {0};
static robot_t g_robot = {0};
static uint32_t shutdown_start_timer;
static uint32_t g_bms_com_timer = 0;

/**
 * 开机按键检测任务
 * @param  _param  : 任务参数
 * @return         : void
 */
void power_key_gpio_init(void)
{
    rt_pin_mode(PIN_POWER_KEY_IN_PE4,        PIN_MODE_INPUT);
    rt_pin_mode(PIN_POWER_12V_KEY_LED_PI9,   PIN_MODE_OUTPUT);
	
//	POWER_12V_KEY_LED_ENABLE;
}

void power_key_sacn(void)
{
	uint8_t power_key_value_new = 0;
	static uint8_t power_key_scan_cnt = 0;
	static uint32_t s_power_key_press_timer;
	static uint8_t s_power_key_release = 0;
	
	if(!POWER_KEY_IN_STATUS)
		power_key_value_new =  1;
	
	if(power_key_value_new != power_key_value)
	{
		power_key_scan_cnt ++;
		if(power_key_scan_cnt > 5)
		{
			power_key_scan_cnt = 0;
			power_key_value = power_key_value_new;
			if(power_key_value)
			{
				s_power_key_press_timer = os_gettime_ms();
			}
			else
			{
				s_power_key_release = 1;
			}
			LOG_W("power_on key value: %d\n", power_key_value);
		}
	}
	
	if((power_key_value) || (s_power_key_release))
	{
		if(s_power_key_release)
		{
			s_power_key_release = 0;
			if((os_gettime_ms() - s_power_key_press_timer) > 3000)	//3s
			{
				//进入关机流程
				g_robot.shutdown_report = YES;
				
				if(g_robot.shutdown_start_by_key == NO)
				{
					power_led_state = 1;
					g_robot.shutdown_start_by_key = YES;
					shutdown_start_timer = os_gettime_ms();
				}
			}
			else if((os_gettime_ms() - s_power_key_press_timer) > 1000)	//1s
			{
				if(g_robot.robot_mode == MODE_STANDBY)
				{
					//结束待机状态
					g_robot.normal_standby_wakeup_report = YES;
					power_wakeup_set();
				}
			} else if (os_gettime_ms() < s_power_key_press_timer) {
            s_power_key_press_timer = os_gettime_ms();
			}
		}
		else if(power_key_value)
		{
			if((os_gettime_ms() - s_power_key_press_timer) > 3000)	//3s
			{
				//进入关机流程
				g_robot.shutdown_report = YES;
				
				if(g_robot.shutdown_start_by_key == NO)
				{
					power_led_state = 1;
					g_robot.shutdown_start_by_key = YES;
					shutdown_start_timer = os_gettime_ms();
				}
			} else if (os_gettime_ms() < s_power_key_press_timer) {
            s_power_key_press_timer = os_gettime_ms();
			}
		}
	}
	
}

static void light_mode_set(uint8_t mode)
{
	g_robot.light_mode = mode;
}
uint8_t light_mode_get(void)
{
	return g_robot.light_mode;
}

static void power_mode_set(uint8_t mode)
{
	g_robot.power_mode = mode;
}
uint8_t power_mode_get(void)
{
	return g_robot.power_mode;
}

uint8_t robot_mode_get(void)
{
	return g_robot.robot_mode;
}

void charger_gpio_init(void)
{
    rt_pin_mode(PIN_CHARGE_AUTO_CTL_PE15,        PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_CHARGE_MANUAL_CTL_PH9,       PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_CHARGE_POLE_CTL_PH11,    PIN_MODE_OUTPUT);
	
	CHARGE_AUTO_CTL_DISABLE;
	CHARGE_MANUAL_CTL_DISABLE;
	CHARGE_POLE_CTL_DISABLE;
}

static uint8_t charger_pole_det_value = 0;
void charger_pole_detect(void)
{
	uint8_t charger_pole_det_value_new = 0;
	static uint8_t charger_pole_det_cnt = 0;
	uint16_t temp;
	
	temp = vol_get_data(VOLDET_CH_CHARGER_POLE);
//	rt_kprintf("charger pole vol_val:%d\n", temp);
	
	if(temp >2000)		//2V
	{
		charger_pole_det_value_new = 0x01;
	}
	
	if(charger_pole_det_value != charger_pole_det_value_new)
	{
		if(charger_pole_det_cnt > 4)	//5
		{
			charger_pole_det_cnt = 0;
			charger_pole_det_value = charger_pole_det_value_new;
			if(charger_pole_det_value)
			{
				rt_memset(&g_udock, 0, sizeof(g_udock));
				g_udock.charger_pole_report = YES;
			}
			else
			{
				g_udock.charger_pole_report = NO;
			}
			LOG_W("charger pole value: %d\n", charger_pole_det_value);
			rt_kprintf("charger pole vol_val:%d\n", temp);
		}
		else
		{
			charger_pole_det_cnt ++;
		}
	}
	
	if((charger_pole_det_value) && (g_udock.udock_pointup_finish ))
	{
		if(!CHARGE_POLE_CTL_STATUS)
		{
			CHARGE_POLE_CTL_ENABLE;
			LOG_W("udock point up, enable pole ctrl.\n");
		}
	}
}

void udock_pointdown_process(void)
{
	if(g_udock.pointdown_start ==  YES)
	{
		LOG_W("udock point down start.\n");
		CHARGE_POLE_CTL_DISABLE;
		CHARGE_AUTO_CTL_DISABLE;
		CHARGE_MANUAL_CTL_DISABLE;
		power_wakeup_set();
		g_udock.pointdown_start = NO;
		g_udock.udock_pointdown_finish = YES;
		LOG_W("udock point down finish.\n");
	}
}

static uint8_t g_charger_in_det_value = 0;
void charger_detect(void)
{
	uint8_t charger_in_det_value_new = 0;
	static uint8_t s_charger_in_det_cnt = 0;
	uint16_t temp1, temp2;
    bat_data_t* pdata = RT_NULL;
	static uint32_t s_charge_start_timer;
	static uint32_t s_charge_vol_print_timer;
    

	
	charger_in_det_value_new = 0;
	temp1 = vol_get_data(VOLDET_CH_MCHARGE_24V);
	temp2 = vol_get_data(VOLDET_CH_ACHARGE_24V);
//	rt_kprintf("charger manual vol_val:%d\t%d\n", temp1, temp2);

	if(temp1 >20000)		//20V
	{
		charger_in_det_value_new |= 0x01;
	}
	if(temp2 >20000)		//20V
	{
		charger_in_det_value_new |= 0x02;
	}
	
	/*if((os_gettime_ms() - s_charge_vol_print_timer)>200)
	{
		s_charge_vol_print_timer = os_gettime_ms();
	} else if (os_gettime_ms() < s_charge_vol_print_timer) {
            s_charge_vol_print_timer = os_gettime_ms();
			}*/
	
	if(g_charger_in_det_value != charger_in_det_value_new)
	{
		if(s_charger_in_det_cnt > 10)	
		{
			s_charger_in_det_cnt = 0;
			rt_kprintf("charger detect vol_val:%d\t%d\n", temp1, temp2);
			
			if(charger_in_det_value_new == 0)
			{
				CHARGE_AUTO_CTL_DISABLE;
				CHARGE_MANUAL_CTL_DISABLE;
				LOG_W("charger out!");
				charger_data.charger_source = 0;
			}
			else
			{
				if(charger_in_det_value_new > 2)	//直充回充同时插入
				{
					CHARGE_AUTO_CTL_DISABLE;
					CHARGE_MANUAL_CTL_DISABLE;
					LOG_W("charger in err!");
					charger_data.charger_source = 0;
				}
				else if(charger_in_det_value_new == 1)
				{
					g_robot.manual_charge_report = YES;
					CHARGE_AUTO_CTL_DISABLE;
					CHARGE_MANUAL_CTL_ENABLE;
					charger_data.charger_source = CHARGER_MANUAL;
					LOG_W("charger in manual!");
					light_standby_set();
//					power_standby_set();
//				    power_wakeup_set();  /*power on the motor to run for the test while charing in manuual*/

				}
				else if(charger_in_det_value_new == 2)
				{
					g_robot.udock_standby_report = YES;
					CHARGE_AUTO_CTL_ENABLE;
					CHARGE_MANUAL_CTL_DISABLE;
					charger_data.charger_source = CHARGER_AUTO;
					LOG_W("charger in udock!");
					light_standby_set();
//					power_standby_set();
				}
				s_charge_start_timer = os_gettime_ms();
			}
			g_charger_in_det_value = charger_in_det_value_new;
		}
		else
		{
			s_charger_in_det_cnt ++;
		}
	}
	
	if(g_charger_in_det_value > 0)
	{
		if((os_gettime_ms() - s_charge_start_timer)>5000)
		{
			pdata = bat_get_data_ptr();
	//		LOG_W("bat status: %d\n", pdata->status);
			if( (pdata->status != BAT_STATUS_CHARGING) && (pdata->status != BAT_STATUS_FULL) )
			{
				if(charger_data.charger_source)
				{
					CHARGE_AUTO_CTL_DISABLE;
					CHARGE_MANUAL_CTL_DISABLE;
					CHARGE_POLE_CTL_DISABLE;
					LOG_W("turn off the charger!");
					charger_data.charger_source = 0;
				}
				
				/*if(charger_data.charger_source == CHARGER_MANUAL)
				{
	//				bms_control(BMS_CTRL_SLEEP);
				}
				else if(charger_data.charger_source == CHARGER_AUTO)
				{
					//
				}*/
			}
		}
		else if(os_gettime_ms() < s_charge_start_timer){
			s_charge_start_timer = os_gettime_ms();
		}
	}
	
}

static uint8_t get_charge_in_status(void)
{
	return g_charger_in_det_value;
}


/**
 * 电源灯控制
 * @param  _param  : 任务参数
 * @return         : void
 */
static void battery_powerled_run()
{
    static uint8_t s_led_status = 0;

    if(power_led_state != 0)
    {

		if(s_led_status != 0)
		{
			POWER_12V_KEY_LED_ENABLE;
			s_led_status = 0;
		}
		else
		{
			POWER_12V_KEY_LED_DISABLE;
			s_led_status = 1;
		}
    }
}

/**
* 开机按键及按键指示灯任务
 * @param  _param  : 任务参数
 * @return         : void
 */
void power_key_detect(void)
{
	static uint32_t power_key_led_timer;
	power_key_sacn();
	
    if(os_gettime_ms() - power_key_led_timer >= 100)
	{
		power_key_led_timer = os_gettime_ms();
		battery_powerled_run();	//100ms
	} else if (os_gettime_ms() < power_key_led_timer) {
        power_key_led_timer = os_gettime_ms();
    }
}

void power_shutdown_set_by_host(void)
{
	if(g_robot.shutdown_start_by_host == NO)
	{
		power_led_state = 1;
		g_robot.shutdown_start_by_host = YES;
		shutdown_start_timer = os_gettime_ms();
	}
}


void light_standby_set(void)	//standby   udock_shutdown
{
	light_mode_set(LIGHT_STANDBY);
	LOG_W("light-stanby.\n");
}

void power_standby_set(void)	//standby   udock_shutdown
{
	if(g_robot.robot_mode != MODE_STANDBY)
	{
		g_robot.robot_mode = MODE_STANDBY;
		power_mode_set(POWER_STANDBY);
		power_into_standby();
		g_robot.motor_power_status = MOTOR_STANDBY;
		LOG_W("power-stanby.\n");
	}
}


void power_wakeup_set(void)
{
	if(g_robot.robot_mode != MODE_WORK)
	{
		g_robot.robot_mode = MODE_WORK;
		power_into_wrok();
		g_robot.motor_power_status = MOTOR_WORK;
		power_mode_set(POWRE_WORK);
		light_mode_set(LIGHT_WORK);
		LOG_W("power-wakeup.\n");
	}

}

static void shutdown_process(void)
{
    bat_data_t* pdata = RT_NULL;

    pdata = bat_get_data_ptr();
	if((pdata->ex_io_status == 0x02) || (pdata->sleep_flag == 0xbb))
	{
		if(g_robot.shutdown_start_by_key == NO)
		{
			power_led_state = 1;
			g_robot.shutdown_start_by_key = YES;
			shutdown_start_timer = os_gettime_ms();
			LOG_W("power_on_key:%02x\t%02x\n", pdata->ex_io_status, pdata->sleep_flag);
		}
	}
	
	if(g_robot.shutdown_start_by_key == YES)
	{
		if(os_gettime_ms() - shutdown_start_timer >= 70000)	//65s
		{
			LOG_W("power_on key: enable bms sleep.\n");
			bms_control(BMS_CTRL_SLEEP);
		}
		else if(os_gettime_ms() < shutdown_start_timer){
			shutdown_start_timer = os_gettime_ms();
		}
	}
	
	if(g_robot.shutdown_start_by_host == YES)
	{
		if(os_gettime_ms() - shutdown_start_timer >= 60000)	//10s
		{
			LOG_W("host: enable bms sleep.\n");
			bms_control(BMS_CTRL_SLEEP);
		}
		else if(os_gettime_ms() < shutdown_start_timer){
			shutdown_start_timer = os_gettime_ms();
		}
	}

}
static void power_management_task_main(void* _param)
{
	static uint8_t s_power_up = NO;
	static uint32_t s_bms_discharger_timer;
	static uint8_t s_power_up_driver = NO;
    bat_data_t* _pdata = RT_NULL;

    _pdata = bat_get_data_ptr();
    while(1)
    {
		
		/*if(_pdata->status == BAT_STATUS_COMERR)
		{
			if(os_gettime_ms() - g_bms_com_timer >= 3000)	//3s
			{
				;
			}
		}
		else
		{
			if(bms_get_wakeup_source() == 0x01)	//开关机按键唤醒BMS开机
			{
				//正常开机打开相应电源
				power_up();
			}
			else if(bms_get_wakeup_source() == 0x02)	//其他原因（充电）唤醒BMS开机
			{
				if(get_charge_in_status() != 0)
				{
					//正常充电唤醒
					power_standby_set();
				}
				else if(os_gettime_ms() - g_bms_com_timer >= 10000)	//10s
				{
					//反电动势充电唤醒
					if(s_power_up_driver == NO)
					{
						power_up_driver();
						s_power_up_driver = YES;
					}
					
					if(_pdata->status ==  )
					
				}
			}
		}*/
		
		if(s_power_up == NO)
		{
			if(bms_get_wakeup_source() == 0x01)	//开关机按键唤醒BMS开机
			{
				//正常开机打开相应电源
				LOG_W("power: power up by key.\n");
				power_up();
				g_robot.motor_power_status = MOTOR_WORK;
				s_power_up = YES;
			}
			else if(bms_get_wakeup_source() == 0x02)	//其他原因（充电）唤醒BMS开机
			{
				if(get_charge_in_status() != 0)
				{
					//正常充电唤醒
					LOG_W("power: power up by charger.\n");
//					power_standby_set();
					power_up();
					g_robot.motor_power_status = MOTOR_WORK;
					s_power_up = YES;
				}
				else if(os_gettime_ms() - g_bms_com_timer >= 10000)	//10s
				{
					//反电动势充电唤醒
					if(s_power_up_driver == NO)
					{
						LOG_W("power: power up by EMF, power up driver.\n");
						power_up_driver();
						g_robot.motor_power_status = MOTOR_WORK;
						s_power_up_driver = YES;
						s_bms_discharger_timer = os_gettime_ms();
					}
					
					if((_pdata->status ==  BAT_STATUS_IDLE) || (_pdata->status ==  BAT_STATUS_DISCHARING))
					{
						if(os_gettime_ms() - s_bms_discharger_timer >= 60000)	//60s内未检测到BMS出于充电状态，则手动关闭BMS放电输出
						{
							LOG_W("no EMF, enable bms sleep.\n");
							bms_control(BMS_CTRL_SLEEP);
						}
					}
					else
					{
						s_bms_discharger_timer = os_gettime_ms();
					}
					
				}
				else if(os_gettime_ms() < g_bms_com_timer){
					g_bms_com_timer = os_gettime_ms();
				}					
			}
			else if(bms_get_wakeup_source() == 0x00)	//MCU与BMS通信异常
			{
				if(os_gettime_ms() - g_bms_com_timer >= 3000)	//3s
				{
					if(s_power_up_driver == NO)
					{
						LOG_W("power: bms comm err, power up by EMF, power up driver.\n");
						power_up_driver();
						g_robot.motor_power_status = MOTOR_WORK;
						s_power_up_driver = YES;
						s_bms_discharger_timer = os_gettime_ms();
					}
				}
				else if(os_gettime_ms() < g_bms_com_timer){
					g_bms_com_timer = os_gettime_ms();
				}
			}
		}
		
		power_key_detect();		//开机按键及按键灯
		charger_pole_detect();		//回充电极片接触检测
		charger_detect();			//充电插入检测
		udock_pointdown_process();	//下桩检测
		shutdown_process();
        rt_thread_mdelay(10);
    }
}

/**
 * 模块初始化
 * @param      : void
 * @return     : void
 */
int32_t power_management_init(void)
{   
	charger_gpio_init();
	power_key_gpio_init();
	g_robot.robot_mode = MODE_WORK;
	g_robot.motor_power_status = MOTOR_DEFAULT;
	g_bms_com_timer = os_gettime_ms();
if(DF_THREAD_STATIC_MEMORY == 0){	
    rt_thread_t thread = rt_thread_create("task_power_management", \
                                  power_management_task_main, \
                                  RT_NULL, \
                                  TASK_STACK_SIZE_POWER_MANAGEMENT, \
                                  TASK_PRIORITY_POWER_MANAGEMENT, \
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
    static struct rt_thread power_management_thread;
    ALIGN(RT_ALIGN_SIZE)
    static char power_management_thread_stack[TASK_STACK_SIZE_POWER_MANAGEMENT]; 
    	rt_err_t result = RT_EOK;
    result = rt_thread_init(&power_management_thread,
                            "task_power_management",
                            power_management_task_main, RT_NULL,
                            &power_management_thread_stack[0], sizeof(power_management_thread_stack),
                            TASK_PRIORITY_POWER_MANAGEMENT, 20);

    if (result == RT_EOK)
    	rt_thread_startup(&power_management_thread);
    else
    	LOG_I("%s thread create failed.",__FUNCTION__);
    
}    
    return 0;
}

uint8_t power_down_get_status_report(void)
{
	if(g_robot.shutdown_report)
	{
		return 1;
	}
	else if(g_robot.udock_standby_report)
	{
		return 2;
	}
	else if(g_robot.normal_standby_wakeup_report)
	{
		return 3;
	}
	else if(g_robot.manual_charge_report)
	{
		return 4;
	}
	else
	{
		return 0;
	}
}
void power_down_clr_status_report(uint8_t status)
{
	if(status == 1)
	{
		g_robot.shutdown_report = NO;
	}
	else if(status == 2)
	{
		g_robot.udock_standby_report = NO;
	}
	else if(status == 3)
	{
		g_robot.normal_standby_wakeup_report = NO;
	}
	else if(status == 4)
	{
		g_robot.manual_charge_report = NO;
	}
}

uint8_t udock_get_status_report(void)
{
	if(g_udock.charger_pole_report)
	{
		return 1;
	}
	else if(g_udock.udock_pointdown_finish)
	{
		return 2;
	}
	else
	{
		return 0;
	}
	
}

void udock_clr_pole_report(void)
{
	g_udock.charger_pole_report = NO;
}

void udock_clr_pointdown_finish(void)
{
	g_udock.udock_pointdown_finish = NO;
}


void udock_set_pointup_finish(void)
{
	g_udock.udock_pointup_finish = YES;
}

void udock_set_pointdown_flag(void)
{
	g_udock.pointdown_start = YES;
}

uint8_t power_motor_status_get(void)
{
	return g_robot.motor_power_status;
}

//---------------------------------------------------------------------------------

#include "finsh.h"

static void udock_debug(uint8_t argc, char **argv)
{
    if(argc < 2 || argc > 2)
    {
        rt_kprintf("Please input: udock_debug <1/0>\n");
    }
    else
    {
        if(atoi(argv[1]) == 1)
		{
			udock_set_pointup_finish();
		}
		else if(atoi(argv[1]) == 2)
		{
			udock_set_pointdown_flag();
		}
		else if(atoi(argv[1]) == 3)
		{
			CHARGE_AUTO_CTL_ENABLE;
		}
		else if(atoi(argv[1]) == 4)
		{
			CHARGE_AUTO_CTL_DISABLE;
		}
		
		else if(atoi(argv[1]) == 5)
		{
			CHARGE_MANUAL_CTL_ENABLE;
		}
		else if(atoi(argv[1]) == 6)
		{
			CHARGE_MANUAL_CTL_DISABLE;
		}
    }
}
MSH_CMD_EXPORT(udock_debug, debug udock);

static void robot_onoff_debug(uint8_t argc, char **argv)
{
    if(argc < 2 || argc > 2)
    {
        rt_kprintf("Please input: robot_onoff_debug <1/0>\n");
    }
    else
    {
        if(atoi(argv[1]) == 1)
		{
			power_standby_set();
			light_standby_set();
		}
		else if(atoi(argv[1]) == 2)
		{
			power_wakeup_set();
		}
    }
}
MSH_CMD_EXPORT(robot_onoff_debug, debug robot onoff process);










