/*
 * HTTP server example.
 *
 * This sample code is in the public domain.
 */

#include <board.h>
#include <string.h>
#include <stdio.h>
#include <httpd.h>
#include "controller.h"
#include <lwip/inet.h>
#include "diagnosis.h"
#include "diagnosis.h"
#include "rt_fota.h"
#include "motor_upgrade.h"
enum {
	UNDEF_WEB_UP,
	CALI_WEB_UP,
	DIAGNOSIS_WEB_UP,
	ODOM_WEB_UP,
	POWER_WEB_UP,
	BRAKE_SENSOR_WEB_UP,
	UPGRADE_WEB_UP,
	OTHER_WEB_UP,
};

volatile struct tcp_pcb *websocket_task_pcb = NULL;
volatile int current_web_index = UNDEF_WEB_UP;

char current_web_name[DF_WEB_NAME_LENGTH];
int getCalibrationProgress(void);
char response[DF_WEB_RES_LENGTH];

void websocket_task(void *pvParameter) {

	printf("creat websocket_task addr\n");

	for (;;) {
		int len = 0;
		rt_thread_mdelay(1000);
		struct tcp_pcb *pcb = (struct tcp_pcb*) websocket_task_pcb;
		if (pcb == NULL || pcb->state != ESTABLISHED) {
			websocket_task_pcb = NULL;
			enterCalibrationWeb(RT_FALSE);
			enterDiagnosisWeb(RT_FALSE);
//            printf("Connection closed, deleting task\n");
			rt_thread_mdelay(1000);

			continue;
		}

		switch (current_web_index) {
		case CALI_WEB_UP:
			len = getCaliJson(response, current_web_name);
			break;
		case DIAGNOSIS_WEB_UP:
			len = getDiagnosisJson(response, current_web_name);
			break;
		case POWER_WEB_UP:
			len = getPowerJson(response, current_web_name);
			break;
		case BRAKE_SENSOR_WEB_UP:
			len = getBrakeSensorJson(response, current_web_name);
			break;
		case ODOM_WEB_UP:
			break;
		case UPGRADE_WEB_UP:
			len = getUpgradeJson(response, current_web_name);
			break;

		case OTHER_WEB_UP:
			len = getOtherJson(response, current_web_name);
			break;

		case UNDEF_WEB_UP:
		default:
			break;
		}
		if ((len < sizeof(response)) && (len > 0)) {
			// printf("cresponse(%d):%s\r\n",strlen(response),response);
			websocket_write(pcb, (unsigned char*) response, len, WS_TEXT_MODE);
		}

	}

}
/**
 * This function is called when websocket frame is received.
 *
 * Note: this function is executed on TCP thread and should return as soon
 * as possible.
 */
#include "priority.h"
void websocket_cb(struct tcp_pcb *pcb, uint8_t *data, u16_t data_len,
		uint8_t mode) {
	//printf("[websocket_callback]:\n%.*s\n", (int) data_len, (char*) data);

	int strint;
	char *group = (char*) data;
	char *cmd = (char*) data;
	if (data_len > 0)
		data[data_len] = 0;
	printf("websocket_cb:%s\n", group);
	while ((*cmd) != ':')
		cmd++;
	*cmd = 0;
	cmd++;
	printf("group:%s cmd:%s\n", group, cmd);

	if (0 == strcmp(group, "cali") || 0 == strcmp(group, "other")) {
		fromControllerWebCmd(group, cmd);
	} else if (!strcmp((char*) data, "pow")) {

	} else if (!strcmp((char*) data, "odom")) {

	} else if (!strcmp((char*) data, "grade")) {

	}

	if (diagnosisCmdfun(group, cmd))
		return;
	if (powerDiaCmdfun(group, cmd))
		return;
	if (powerBrakeSensorCmdfun(group, cmd))
		return;
	if (motorUpgradeCmdfun(group, cmd))
		return;
	return;

}

/**
 * This function is called when new websocket is open and
 * creates a new websocket_task if requested URI equals '/stream'.
 */
#define  TASK_STACK_SIZE_WEBSOCKET     (2048)  
#define TASK_PRIORITY_WEBSOCKET 48

int websocket_open_cb(struct tcp_pcb *pcb, const char *uri) {

	//  printf("ws uri addr: %s port:%d\n", inet_ntoa(pcb->remote_ip.addr),pcb->remote_port);
	printf("WS URI: %s\n", uri);
	strcpy(current_web_name, uri);
	if (!strcmp(uri, "/cali")) {
		printf("request for streaming  %s\n", uri);
		current_web_index = CALI_WEB_UP;
		websocket_task_pcb = pcb;
		enterCalibrationWeb(RT_TRUE);
		printf("request for streaming  %s\n", uri);
		return 0;
	} else if (!strcmp(uri, "/dia")) {
		printf("request for streaming  %s\n", uri);
		current_web_index = DIAGNOSIS_WEB_UP;
		websocket_task_pcb = pcb;
		enterDiagnosisWeb(RT_TRUE);
		printf("request for streaming  %s\n", uri);
		return 0;
	} else if (!strcmp(uri, "/bra")) {
		printf("request for streaming  %s\n", uri);
		current_web_index = BRAKE_SENSOR_WEB_UP;
		websocket_task_pcb = pcb;
		printf("request for streaming  %s\n", uri);
		return 0;
	} else if (!strcmp(uri, "/pow")) {
		printf("request for streaming  %s\n", uri);
		current_web_index = POWER_WEB_UP;
		websocket_task_pcb = pcb;
		printf("request for streaming  %s\n", uri);
		return 0;
	} else if (!strcmp(uri, "/upgrade")) {
		printf("request for streaming  %s\n", uri);
		current_web_index = UPGRADE_WEB_UP;
		websocket_task_pcb = pcb;
		printf("request for streaming  %s\n", uri);
		return 0;
	} else if (!strcmp(uri, "/odom")) {
		printf("request for streaming  %s\n", uri);
		current_web_index = ODOM_WEB_UP;
		websocket_task_pcb = pcb;
		printf("request for streaming  %s\n", uri);
		return 0;
	} else if (!strcmp(uri, "/upg")) {
		printf("request for streaming  %s\n", uri);
		current_web_index = UPGRADE_WEB_UP;
		websocket_task_pcb = pcb;
		printf("request for streaming  %s\n", uri);
		return 0;
	} else if (!strcmp(uri, "/other")) {
		printf("request for streaming  %s\n", uri);
		current_web_index = OTHER_WEB_UP;
		websocket_task_pcb = pcb;
//		printf("request for streaming  %s\n", uri);
		return 0;
	}
}

enum {
	SSI_HARD_VERSION, SSI_SOFT_VERSION, SSI_BUILD_TIM, SSI_BATTERY_INFO
};

const char *pcConfigSSITags[] = { "harver", // SSI_UPTIME
		"sofver",   // SSI_FREE_HEAP
		"buitim",     // SSI_LED_STATE
		"batteryinfo"     // SSI_LED_STATE
		};

int32_t ssi_handler(int32_t iIndex, char *pcInsert, int32_t iInsertLen) {
	//printf("%s:%d\r\n",__FUNCTION__,iIndex);
	switch (iIndex) {
	case SSI_HARD_VERSION:
		snprintf(pcInsert, iInsertLen, "%d  \r\n", hardware_version_get());
		break;
	case SSI_SOFT_VERSION: {
		fota_flags_t *pflags = fota_flags_get();
		snprintf(pcInsert, iInsertLen, "  %s  \r\n", pflags->app_version);
	}
		break;
	case SSI_BUILD_TIM:
		snprintf(pcInsert, iInsertLen, " %s  |  %s  \r\n", __DATE__, __TIME__);
		// printf("%s:%s\r\n",__FUNCTION__,pcInsert);
		break;
	case SSI_BATTERY_INFO:
		snprintf(pcInsert, iInsertLen, 1 ? "Off" : "On");
		break;
	default:
		snprintf(pcInsert, iInsertLen, "N/A");
		break;
	}

	/* Tell the server how many characters to insert */
	return (strlen(pcInsert));
}

void httpd_task(void *pvParameters) {

	http_set_ssi_handler((tSSIHandler) ssi_handler, pcConfigSSITags,
			sizeof(pcConfigSSITags) / sizeof(pcConfigSSITags[0]));
	websocket_register_callbacks((tWsOpenHandler) websocket_open_cb,
			(tWsHandler) websocket_cb);
	httpd_init();

}

int user_init(void) {
	if (DF_THREAD_STATIC_MEMORY == 0) {
		rt_thread_t thread = rt_thread_create("websocket_task", websocket_task,
				(void*) NULL, TASK_STACK_SIZE_WEBSOCKET,
				TASK_PRIORITY_WEBSOCKET, 10);

		if (thread != RT_NULL) {
			rt_thread_startup(thread);
			printf("request for streaming ok\n");
		} else {
			printf("request for streaming false\n");

		}
	} else {
		static struct rt_thread websocket_thread;
		ALIGN(RT_ALIGN_SIZE)
		static char websocket_thread_stack[TASK_STACK_SIZE_WEBSOCKET];
		rt_err_t result = RT_EOK;
		result = rt_thread_init(&websocket_thread, "websocket_task",
				websocket_task, RT_NULL, &websocket_thread_stack[0],
				sizeof(websocket_thread_stack),
				TASK_PRIORITY_WEBSOCKET, 10);

		if (result == RT_EOK)
			rt_thread_startup(&websocket_thread);
		else
			;
		//LOG_I("odom thread create failed.");

	}

	httpd_task(0);
	// xTaskCreate(&httpd_task, "HTTP Daemon", 128, NULL, 2, NULL);
}
