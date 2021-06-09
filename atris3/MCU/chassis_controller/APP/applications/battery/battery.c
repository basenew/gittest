/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : battery.c
 * Brief      : 电池信息采集

 * Change Logs
 * Date           Author          Notes
 * 2020-09-05     wuxiaofeng      first version        
 */
#include "common.h"
#include "app_cfg.h"
//#include "can_cmd.h"
//#include "can_service.h"
//#include "can_notify.h"
//#include "key.h"
#include "bms.h"
#include "battery.h"
#include "can1.h"

#define LOG_TAG "battery"
#define LOG_LVL LOG_LVL_INFO
#include <rtdbg.h>


static void charger_check(void);
//static void data_report(void);


static bat_t bat_obj;

int32_t battery_msg_post(int32_t idx, int32_t *pbuf)
{
    bat_msg_t msg;

    if (idx >= BAT_MSG_IDX_MAX) {
        return -1;
    }
    rt_memset(&msg, 0, sizeof(bat_msg_t));
    msg.idx = idx;
    if (pbuf != RT_NULL) {
        rt_memcpy(msg.buff, pbuf, sizeof(msg.buff));
    }
    return (int32_t)rt_mq_send(bat_obj.mq, &msg, sizeof(bat_msg_t));
}

static void handle_msg(bat_msg_t *_pmsg)
{
    char version[80];
	
    switch (_pmsg->idx)
    {
        case BAT_MSG_IDX_CTRL:
        {
            bms_ctrl_t cmd = (bms_ctrl_t)_pmsg->buff[0];
            bms_control(cmd);
        } break;

        case BAT_MSG_IDX_SYNC:
        {
            bms_sync();
        } break;

        case BAT_MSG_IDX_GETLOG:
        {
           // bms_getlog();
        } break;

        case BAT_MSG_IDX_GETVER:
        {
            bms_getversion(E_BMS_VER_SOFT);
            bms_getversion(E_BMS_VER_HARD);
			get_bms_sw_version(version);
            rt_kprintf("bms softverson: %s\n", version);
			rt_memset(version, 0, sizeof(version));
			get_bms_hw_version(version);
            rt_kprintf("bms hardverson: %s\n", version);
        } break;

        case BAT_MSG_IDX_BMS_NOTIFY:
        {

        } break;
        
        case BAT_MSG_INDEX_KEY:
        {

        } break;

        default: break;
    }
}



static void task_main(void *param)
{
    bat_msg_t msg;
    uint32_t s_timer1= 0;
	
	battery_msg_post(BAT_MSG_IDX_GETVER, RT_NULL);

    while (1)
    {
        if (rt_mq_recv(bat_obj.mq, &msg, sizeof(bat_msg_t), MS_TO_TICKS(200)) == RT_EOK)
        {
            handle_msg(&msg);
        }

        if (os_gettime_ms() - s_timer1 >= BAT_SAMPLE_PERIOD)
        {
            s_timer1 = os_gettime_ms();
            battery_msg_post(BAT_MSG_IDX_SYNC, RT_NULL);
//            data_report();
        } else if (os_gettime_ms() < s_timer1) {
            s_timer1 = os_gettime_ms();
        }
        
        charger_check();
    }
}

int32_t battery_init(void)
{
    rt_thread_t tid;

    if (bms_init() < 0) {
        return -1;
    }
	CAN1_init();

    bat_obj.mq = rt_mq_create("battery", sizeof(bat_msg_t), 8, RT_IPC_FLAG_FIFO);
    if (bat_obj.mq == RT_NULL) {
        return -1;
    }

if(DF_THREAD_STATIC_MEMORY == 0){    
    
    tid = rt_thread_create("battery",task_main, RT_NULL,
                           TASK_STACK_SIZE_BAT, TASK_PRIORITY_BAT, 20);
    if (tid != RT_NULL) {
        rt_thread_startup(tid);
        return 0;
    }
    else {
        rt_mq_delete(bat_obj.mq);
        return -1;
    }
    
}else{
    static struct rt_thread battery_thread;
    ALIGN(RT_ALIGN_SIZE)
    static char battery_thread_stack[TASK_STACK_SIZE_BAT]; 
    	rt_err_t result = RT_EOK;
    result = rt_thread_init(&battery_thread,
                            "battery",
                            task_main, RT_NULL,
                            &battery_thread_stack[0], sizeof(battery_thread_stack),
                            TASK_PRIORITY_BAT, 20);

    if (result == RT_EOK){
    	rt_thread_startup(&battery_thread);
        return 0;
    }else {
        rt_mq_delete(bat_obj.mq);
        LOG_I("%s thread create failed.",__FUNCTION__);
        return -1;
    }
    	
    
}
}

static void report_charger_source(void)
{
    /*
    uint8_t txbuf[8] = {0};
    
    //机器结构上能够保证回充插头和适配器插头不会同时插入
    if (bat_obj.charger.adapter.status == DF_IN)
    {
        bat_obj.charger.charge_source = SOURCE_ADAPTER;
    }
    else if (bat_obj.charger.base.status == DF_IN)
    {
        bat_obj.charger.charge_source = SOURCE_BASE;
    }
    else
    {
        bat_obj.charger.charge_source = SOURCE_IDLE;
    }    
  
    txbuf[0] = CANCMD_QUERY_CHARGE_SOURCE + 1;
    txbuf[1] = bat_obj.charger.charge_source;
    can_notify_msg(IDX_CAN1, NODE_ID, CAN_RETURN_MODE, txbuf, 2);
    */
}




static void charger_check(void)
{
    enum
    {
        STATE_OUT = 0,
        STATE_IN,
    };
    
    static uint8_t s_state = STATE_OUT;
    static uint32_t s_timer1 = 0;    

    
    switch (s_state)
    {
    case STATE_OUT:
        if ((bat_obj.charger.adapter.status == IN && os_gettime_ms() - bat_obj.charger.adapter.moment >= 500) \
            || (bat_obj.charger.base.status == IN && os_gettime_ms() - bat_obj.charger.base.moment >= 500))
        {
            s_state = STATE_IN;
            report_charger_source();
        }
        else
        {
            ;
        }
        break;

    case STATE_IN:
        if (bat_obj.data.status == BAT_STATUS_FULL || bat_obj.data.soc == 100)
        {
            bat_obj.charger.status = BAT_CHARGER_STATUS_FULL;
        }
        else {
            bat_obj.charger.status = BAT_CHARGER_STATUS_ING;
        }
        
        if (bat_obj.charger.adapter.status == OUT && bat_obj.charger.base.status == OUT)
        {
            bat_obj.charger.status = BAT_CHARGER_STATUS_IDLE;
            s_timer1 = 0;
            s_state = STATE_OUT;
            report_charger_source();
        }
        else
        {
            if (os_gettime_ms() - s_timer1 >= 1000)
            {
                s_timer1 = os_gettime_ms();
                report_charger_source();
            } else if (os_gettime_ms() < s_timer1) {
                s_timer1 = os_gettime_ms();
            }
        }
        
        break;

    default:
        break;
    }
    
    static uint8_t s_charger_status_bk = 0xFF;
    if(bat_obj.charger.status != s_charger_status_bk) {
        s_charger_status_bk = bat_obj.charger.status;
        LOG_W("charger status: %d", bat_obj.charger.status);
    }
}

/*
static void halt_bms_bycan(uint8_t _node_id, const uint8_t *_prxbuffer, uint8_t *_ptxbuffer, uint8_t *_ptxmsg_len)
{
	if (_ptxbuffer == RT_NULL)
	{
		return;
	}
	send_can_msg(BAT_CAN_MSG_HALT);
    _ptxbuffer[0] = CANCMD_BATTERY_ENTER_HALT + 1;
    *_ptxmsg_len = 1;
}
*/

/*
static void shutdown_bms_bycan(uint8_t _node_id, const uint8_t *_prxbuffer, uint8_t *_ptxbuffer, uint8_t *_ptxmsg_len)
{
	if (_ptxbuffer == RT_NULL)
	{
		return;
	}
	send_can_msg(BAT_CAN_MSG_SHUTDOWN);
    _ptxbuffer[0] = CANCMD_BATTERY_ENTER_SHUTDOWN + 1;
    *_ptxmsg_len = 1;
}
*/

/*
static void query_charge_source_bycan(uint8_t _node_id, const uint8_t *_prxbuffer, uint8_t *_ptxbuffer, uint8_t *_ptxmsg_len)
{
    if (_ptxbuffer == RT_NULL)
    {
        return;
    }

    _ptxbuffer[0] = CANCMD_QUERY_CHARGE_SOURCE + 1;
    _ptxbuffer[1] = bat_obj.charger.charge_source;
    *_ptxmsg_len = 2;
}
*/

int32_t bat_is_refuse_to_startup(void)
{
    /*
    if (bat_obj.data.soc <= BAT_SOC_LEVEL1) {
        //LOG_W("SOC is too low! %d", bat_obj.data.soc);
        return -1;
    }*/
    /*
    if (batinfo.cell_min <= BAT_CELLVOLT_LEVEL1) {
        LOG_W("CELL_MIN is too low! %d", info.cell_min);
        return -1;
    }
    */
    return 0;
}

int32_t bat_get_data(bat_data_t *pdata)
{
    if (pdata == RT_NULL)
        return -RTN_ERR;

    rt_memcpy(pdata, &(bat_obj.data), sizeof(bat_data_t));
    return RTN_OK;
}

bat_data_t* bat_get_data_ptr(void)
{
    return &bat_obj.data;
}

int32_t bat_get_charger(charger_t *pcharger)
{
    if (pcharger == RT_NULL)
        return -RTN_ERR;

    rt_memcpy(pcharger, &(bat_obj.charger), sizeof(charger_t));
    return RTN_OK;
}

void bat_set_status(uint8_t sta)
{
    bat_data_t* pdata = bat_get_data_ptr();
    pdata->status = sta;
}

