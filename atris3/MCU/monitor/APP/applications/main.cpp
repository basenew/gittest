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
#include "spi_flash_init.h"
#include "log.h"
#include "msg_canpkg.h"
#include "led.h"
#include "hw_wdg.h"
#include "rt_fota.h"
#include "ota.h"
#include "msg_canpkg_app.h"
#include "power_ctrl.h"
#include "utility.h"
#include "voltage_detect.h"
#include "sensor_detec.h"
#include "fan.h"
#include "sonar.h"
#include "light_effect.h"

#define LOG_TAG              "main"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

static void app_version_print(void) {
	fota_flags_t *pflags = fota_flags_get();

	rt_kprintf("\r\n");
	rt_kprintf("\r\n");
	rt_kprintf("------------------------------------------------------\r\n");
	rt_kprintf("             APP BY UBTECH            \r\n");
	rt_kprintf("   Version: %d  |  %s  \r\n", hardware_version_get(), pflags->app_version);
	rt_kprintf("   Build  : %s  |  %s  \r\n", __DATE__, __TIME__);
	rt_kprintf("------------------------------------------------------\r\n");rt_kprintf("\r\n");
}

static int app_env_init(void) {
	canpkg_publisher_init();
	canpkg_subscriber_init();
	return 0;
}

static int app_modules_init(void) {
	ota_init();
	power_ctrl_init();
	canpkg_app_init();
	voltage_detect_init();
	sensor_detec_init();
	sonar_init();
	fan_init();
	light_effect_init();
	utility_init();
	return 0;
}

int main(void) {
	spi_flash_init();

	log_init();

	/*ensure threads initialized before main start up*/
	rt_thread_mdelay(500);

	app_version_print();

	app_env_init();

	app_modules_init();
}

//------------------------------------------------
static void version_app(uint8_t argc, char **argv) {
	if (argc != 1) {
		rt_kprintf("Please input: version_app\n");
	} else {
		app_version_print();
	}
}
MSH_CMD_EXPORT(version_app, version_app);

