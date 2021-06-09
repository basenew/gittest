/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : bms.c
 * Brief      : 冠宇电池通信协议

 * Change Logs
 * Date           Author          Notes
 * 2020-08-20     wuxiaofeng      first version        
 */
#include "rtthread.h"
#include "rtdevice.h"
#include "common.h"
#include "app_cfg.h"
#include "led.h"
#include "battery.h"
#include "bms.h"
#include "can1.h"

#define LOG_TAG "bms"
#define LOG_LVL LOG_LVL_INFO
#include <rtdbg.h>


#define EVENT_PRESYNC (1 << 0)
#define EVENT_SYNC    (1 << 1)
#define EVENT_PRELOG  (1 << 2)
#define EVENT_LOG     (1 << 3)
#define EVENT_CTRL    (1 << 4)
#define EVENT_VER_SOFT (1 << 5)
#define EVENT_VER_HARD (1 << 6)



#define WAIT_MSG_TIMEOUT_SYNC (500)
#define WAIT_MSG_TIMEOUT_PRELOG (300)
#define WAIT_MSG_TIMEOUT_LOG  (300)
#define WAIT_MSG_TIMEOUT_CTRL (300)
#define WAIT_MSG_TIMEOUT_VER (2000)

//static int32_t can_send(uint32_t id, const uint8_t *txbuf, uint8_t txlen);

//typedef rt_err_t (*rx_call_t)(rt_device_t dev, rt_size_t size);

//struct can_app_struct
//{
//    const char *name;
//    rt_device_t device;
//    uint32_t baudrate;
//    struct rt_can_filter_config *filter;
//    rt_sem_t rx_sem;
//    rx_call_t rx_call;
//};

//static struct can_app_struct can_data;

//static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
//{
//    rt_sem_release(can_data.rx_sem);
//    return RT_EOK;
//}

//static struct can_app_struct can_data = {
//    .name = "can1",
//    .device = RT_NULL,
//    .baudrate = CAN500kBaud,
//    .filter = RT_NULL, //&filter1,
//    .rx_sem = RT_NULL,
//    .rx_call = can_rx_call,
//};

static struct rt_event msg_event;
static bms_data_t BmsData = {0};
//static bms_log_t BmsLog = {0};
static bms_ver_t BmsVersion = {0};

static int32_t g_debug_print_info = 0;
static int32_t g_debug_raw_print_info = 0;
//static int32_t g_debug_print_log = 0;

static uint8_t g_bms_rx_data_flag = NO;
static CanRxMsg bms_can_rxmsg;


static void debug_info_raw_print(void);
static void debug_info_decoded_print(void);
static int32_t get_version(uint8_t* pdata, _ver_t* pver);
static void bmd_data_deal(CanRxMsg rxmsg);

static void task_main(void *param)
{
//    struct rt_can_msg rx_msg;

//    struct can_app_struct *canpara = &can_data;
    while (1)
    {
        if (g_bms_rx_data_flag == YES)
        {
			bmd_data_deal(bms_can_rxmsg);
			g_bms_rx_data_flag = NO;
        }
		rt_thread_delay(10);
    }
}

//static int32_t device_init(struct can_app_struct *_dev)
//{
//    if (_dev == RT_NULL)
//        return -1;
//    if (_dev->name == RT_NULL)
//        return -1;

//    rt_device_t can_dev = rt_device_find(_dev->name);
//    if (can_dev != RT_NULL && rt_device_open(can_dev, (RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX)) == RT_EOK)
//    {
//        _dev->device = can_dev;
//        _dev->rx_sem = rt_sem_create("bms", 0, RT_IPC_FLAG_FIFO);
//        if (_dev->rx_sem != RT_NULL)
//        {
//            rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)_dev->baudrate);
//            if (rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, _dev->filter) == RT_EOK)
//            {
//                rt_device_set_rx_indicate(can_dev, _dev->rx_call);
//                return 0;
//            }
//            else
//            {
//                LOG_E("fail to set filter, device:%s\n", _dev->name);
//            }
//        }
//    }
//    LOG_E("fail to open device:%s\n", _dev->name);
//    return -1;
//}

int32_t bms_init(void)
{
    rt_event_init(&msg_event, "bms", RT_IPC_FLAG_FIFO);    
if(DF_THREAD_STATIC_MEMORY == 0){   
    rt_thread_t tid;

    tid = rt_thread_create("bms",task_main, RT_NULL,
                           TASK_STACK_SIZE_BMSCAN, TASK_PRIORITY_BMSCAN, 20);
    if (tid != RT_NULL) {
        rt_thread_startup(tid);
        return 0;
    }
}else{
    static struct rt_thread bms_thread;
    ALIGN(RT_ALIGN_SIZE)
    static char bms_thread_stack[TASK_STACK_SIZE_BMSCAN]; 
    	rt_err_t result = RT_EOK;
    result = rt_thread_init(&bms_thread,
                            "bms",
                            task_main, RT_NULL,
                            &bms_thread_stack[0], sizeof(bms_thread_stack),
                            TASK_PRIORITY_BMSCAN, 20);

    if (result == RT_EOK){
    	rt_thread_startup(&bms_thread);
        return 0;
    }else
    	LOG_I("bms thread create failed.");
    
}    
    
    return -1;
}

//static int32_t can_send(uint32_t id, const uint8_t *txbuf, uint8_t txlen)
//{
//    struct rt_can_msg txmsg;
//    struct can_app_struct *canpara = &can_data;
//    uint32_t txsize = 0;

//    if (canpara->device == RT_NULL) {
//        return -1;
//    }

//    if (txlen == 0 || txlen > 8) {
//        LOG_D("txlen error.\n");
//        return -1;
//    }

//    rt_memset(&txmsg, 0, sizeof(txmsg));
//    txmsg.id = id;
//    txmsg.ide = RT_CAN_EXTID;
//    txmsg.rtr = RT_CAN_DTR;
//    txmsg.len = txlen;
//    rt_memcpy(txmsg.data, txbuf, txlen);

//    txsize = rt_device_write(canpara->device, 0, &txmsg, sizeof(txmsg));
//    if (txsize == sizeof(txmsg)) {
//        //if (g_can1_data_print_flag != 0)
////        {
////            rt_kprintf("CAN1Tx: ID-%08X DLC-%02X Data-%02X %02X %02X %02X %02X %02X %02X %02X\n",
////                       txmsg.id, txmsg.len, txmsg.data[0], txmsg.data[1], txmsg.data[2], txmsg.data[3], txmsg.data[4], txmsg.data[5], txmsg.data[6], txmsg.data[7]);
////        }
//        return 0;
//    }
//    else {
//        return -1;
//    }
//}
/*
static void debug_pint_data(int32_t en, uint8_t* pbuf, int32_t size)
{
    if (en == 0) return;

    for (int32_t i = 0; i < size; i++)
    {
        rt_kprintf("%02X ", pbuf[i]);
    }
    rt_kprintf("\r\n");
}
*/
void copy_datum(CanRxMsg rxmsg)
{
	rt_memcpy(&bms_can_rxmsg, &rxmsg, sizeof(rxmsg));
	g_bms_rx_data_flag = YES;
	
//	LOG_D("bms rx data!\n");
	
}
static void bmd_data_deal(CanRxMsg rxmsg)
{
    uint32_t id = 0;
    uint8_t  len = 0;
    uint8_t  data[8] = {0};
    bms_data_t* pbms_data = RT_NULL;

    pbms_data = &BmsData;
    id = rxmsg.ExtId;
    len = rxmsg.DLC;
    rt_memcpy(data, rxmsg.Data, len);

    switch(id)
    {
        case 0x1830B0A0:
            pbms_data->TotalVolt = BYTETOSHORT(data[1], data[0]);
            pbms_data->TotalCurr = BYTETOSHORT(data[3], data[2]);
            pbms_data->SOC = BYTETOSHORT(data[5], data[4]);
            pbms_data->SOH = BYTETOSHORT(data[7], data[6]);
            rt_event_send(&msg_event, EVENT_SYNC);
        break;

        case 0x1831B0A0:
            pbms_data->PackVolt = BYTETOSHORT(data[1], data[0]);
            pbms_data->PackCap = BYTETOSHORT(data[3], data[2]);
			pbms_data->wakeup_source = data[6];
//			rt_kprintf("bms wakeup: %d\n", pbms_data->wakeup_source);

        break;
    
        case 0x1832B0A0:
            pbms_data->CellVolt_High = BYTETOSHORT(data[1], data[0]);
            pbms_data->CellVolt_Low = BYTETOSHORT(data[3], data[2]);
            pbms_data->PackNo_CellVoltHigh = data[4];
            pbms_data->PackNo_CellVoltLow = data[5];
            pbms_data->BunchNo_CellVoltHigh = data[6];
            pbms_data->BunchNo_CellVoltLow = data[7];
        break;

        case 0x1833B0A0:
            pbms_data->CellTempHigh = data[0];
            pbms_data->CellTempLow = data[1];

            pbms_data->PackNo_CellTempHigh = data[4];
            pbms_data->PackNo_CellTempLow = data[5];
            pbms_data->BunchNo_CellTempHigh = data[6];
            pbms_data->BunchNo_CellTempLow = data[7];
        break;

        case 0x1834B0A0:
            /*注：如果外部 IO 状态指示为关机状态时，则主动上传该帧报文 10 次，每帧间隔100ms。*/
            pbms_data->BmsLife = data[0];
            pbms_data->ChgMosStatus = data[1];
            pbms_data->DisChgMosStatus = data[2];
            pbms_data->ExIoStatus = data[3];
            pbms_data->ChgDisChg_Cnts = BYTETOSHORT(data[5], data[4]);
            pbms_data->SleepFlag = data[6];
            pbms_data->ChgStatus = data[7];
//			rt_kprintf("[ChgStatus]%d\n",pbms_data->ChgStatus);
        break;

        case 0x1437B0A0:
            rt_memcpy(pbms_data->uWarn.Warn, data, 8);
        break;

        case 0x1801B0A0:
            pbms_data->CellVolt[0] = BYTETOSHORT(data[1], data[0]);
            pbms_data->CellVolt[1] = BYTETOSHORT(data[3], data[2]);
            pbms_data->CellVolt[2] = BYTETOSHORT(data[5], data[4]);
            pbms_data->CellVolt[3] = BYTETOSHORT(data[7], data[6]);
        break;

        case 0x1802B0A0:
            pbms_data->CellVolt[4] = BYTETOSHORT(data[1], data[0]);
            pbms_data->CellVolt[5] = BYTETOSHORT(data[3], data[2]);
            pbms_data->CellVolt[6] = BYTETOSHORT(data[5], data[4]);
            pbms_data->CellVolt[7] = BYTETOSHORT(data[7], data[6]);
        break;

        case 0x1803B0A0:
            pbms_data->CellVolt[8] = BYTETOSHORT(data[1], data[0]);
            pbms_data->CellVolt[9] = BYTETOSHORT(data[3], data[2]);
            pbms_data->CellVolt[10] = BYTETOSHORT(data[5], data[4]);
            pbms_data->CellVolt[11] = BYTETOSHORT(data[7], data[6]);
        break;

        case 0x1804B0A0:
            pbms_data->CellVolt[12] = BYTETOSHORT(data[1], data[0]);
            pbms_data->CellVolt[13] = BYTETOSHORT(data[3], data[2]);
            pbms_data->CellVolt[14] = BYTETOSHORT(data[5], data[4]);
            pbms_data->CellVolt[15] = BYTETOSHORT(data[7], data[6]);
        break;

        case 0x1820B0A0:
            rt_memcpy(&(pbms_data->CellTemp[0]), data, 8);
        break;

        case 0x1821B0A0:
            rt_memcpy(&(pbms_data->CellTemp[8]), data, 8);
        break;

        //----------------------------------------------------------------------------------------------

        case 0x0CB7B0EA:
            rt_event_send(&msg_event, EVENT_CTRL);
        break;
        
        //----------------------------------------------------------------------------------------------
/*
        case 0x1460B0A0:
            if (data[0] == 0xE1) {
                BmsLog.tar_cnts = BYTETOSHORT(data[7], data[6]);
                rt_event_send(&msg_event, EVENT_PRELOG);
            }
            //debug_pint_data(g_debug_print_log, data, 8);
        break;

        case 0x1462B0A0:
            if (BmsLog.cnts < BMS_LOG_CNTS_MAX) {
                rt_memcpy(&BmsLog.info[BmsLog.cnts].data[BmsLog.idx++], data, 8);
                if (BmsLog.idx >= 4) {
                    BmsLog.idx = 0;
                    BmsLog.cnts ++;
                }
                rt_event_send(&msg_event, EVENT_LOG);
            } else {
                LOG_W("log cnts overflow!\n");
            }
        break;

        //----------------------------------------------------------------------------------------------
        case 0x14E1B0A0:
            if (data[0] == E_BMS_VER_SOFT) {
                BmsVersion.soft.frame_cnts = data[1];
                BmsVersion.soft.size = data[2];
                
            }
            else if (data[0] == E_BMS_VER_HARD) {
                BmsVersion.hard.frame_cnts = data[1];
                BmsVersion.hard.size = data[2];    
                
            }
            else {
                ;
            }
            
        break;
*/
        case 0x14E2B0A0:
            if (data[0] == E_BMS_VER_SOFT)
            {
                if (get_version(data, &BmsVersion.soft) == 0) {
                    rt_event_send(&msg_event, EVENT_VER_SOFT);
                }
                
            }
            else if (data[0] == E_BMS_VER_HARD) {
                if (get_version(data, &BmsVersion.hard) == 0) {
                    rt_event_send(&msg_event, EVENT_VER_HARD);
                }
            }
            else {
                ;
            }
        break;

        default: break;
    }
}

static int32_t get_version(uint8_t* pdata, _ver_t* pver)
{
    uint8_t f_no = 0;
    uint8_t size_cnt = 0;
    int32_t ret = -1;
    
    f_no = pdata[1]; 
    if (f_no < 1) {
        f_no = 1;
    }
    else
    {
        size_cnt = 6*(f_no-1);
        if (size_cnt > sizeof(pver->version) - 1 - 6) {
            size_cnt = sizeof(pver->version) - 1 - 6;
        }
        rt_memcpy(&pver->version[size_cnt], pdata+2, 6);
        if (size_cnt + 6 >= pver->size) {
           ret = 0;
        }
    }
    return ret;
}

static void decode_data(uint8_t commerr)
{
    bat_data_t* pdata = RT_NULL;
    
    pdata = bat_get_data_ptr();
    
    if (commerr == 0) 
    {
        if (pdata->status != BAT_STATUS_COMERR) {
            pdata->status = BAT_STATUS_COMERR;
            LOG_W("bms comm err!");
//            led_set_errcode(LED_ERR_BATCOMM, 1);
        }
        
        pdata->volt_01V = 0;
        pdata->curr_01A = 0;
        pdata->soc = 0xFF;
        pdata->cellvolt_low_mV = 0xFFFF;
        pdata->temp_high = 0xFF;
    }
    else 
    {
        if (pdata->status == BAT_STATUS_COMERR) {
            LOG_W("bms comm success!");
//            led_set_errcode(LED_ERR_BATCOMM, 0);
        }
        
//	rt_kprintf("bms status: %d\n", BmsData.ChgStatus);
        if (BmsData.ChgStatus == 0x00) 
        {
            if (pdata->curr_01A == 0) {
                pdata->status = BAT_STATUS_IDLE;
            }
            else {
               pdata->status = BAT_STATUS_DISCHARING;
            }
        }
        else if (BmsData.ChgStatus == 0x01) {
            pdata->status = BAT_STATUS_CHARGING;
        }
        else if (BmsData.ChgStatus == 0x02) {
            pdata->status = BAT_STATUS_FULL;
        }
        else {
            ;
        }
        
        pdata->volt_01V = BmsData.TotalVolt;
        pdata->curr_01A = BmsData.TotalCurr - 5000;
        pdata->soc = BmsData.SOC;
        pdata->cellvolt_low_mV = BmsData.CellVolt_Low;
        pdata->temp_high = BmsData.CellTempHigh;
		pdata->ex_io_status = BmsData.ExIoStatus;
		pdata->sleep_flag = BmsData.SleepFlag;
    }
}


int32_t bms_control(bms_ctrl_t idx)
{
    uint8_t retry = 5;
    uint32_t e = 0;
    uint32_t id = 0;
    uint8_t txbuf[8] = {0xFF};

    id = 0x0CB0EAB0;
    if (idx == BMS_CTRL_SHUTDOWN) {
        txbuf[0] = 0xAA;
    }
    if (idx == BMS_CTRL_SLEEP) {
        txbuf[0] = 0xBB;
    }

    //clear events first
    rt_event_recv(&msg_event, EVENT_CTRL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &e);

    do
    {
        retry--;
        if (CAN1_Send_bms(id, txbuf, 8) < 0) {
            LOG_D("can send msg failed!\n");
            continue;
        }

        if (rt_event_recv(&msg_event, EVENT_CTRL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, \
            MS_TO_TICKS(WAIT_MSG_TIMEOUT_CTRL), &e) != RT_EOK)
        {
            LOG_D("recv event timeout!\n");
            continue;
        } 
        else {
            //LOG_D("bms responses success!\n");
            break;
        }
    } while(retry);

    return (retry == 0 ? -1 : 0);
}


int32_t bms_sync(void)
{
    int32_t ret = -1;
    uint8_t retry = 5;
	static uint8_t retry_group = 0;
    uint32_t e = 0;
    uint32_t id = 0;
    uint8_t txbuf[8] = {0};

    time_t now;
    struct tm *p_tm;
    struct tm tm_new;

    now = time(RT_NULL);
    rt_enter_critical();
    p_tm = localtime(&now);
    rt_memcpy(&tm_new, p_tm, sizeof(struct tm));
    rt_exit_critical();

    id = 0x14F0EAB0;
    txbuf[0] = 0xAA;
    txbuf[1] = 0x00;
    if (tm_new.tm_year + 1900 < 2000) {
        txbuf[2] = 0;
    } else {
        txbuf[2] = tm_new.tm_year + 1900 - 2000;
    }
    txbuf[3] = tm_new.tm_mon + 1;
    txbuf[4] = tm_new.tm_mday;
    txbuf[5] = tm_new.tm_hour;
    txbuf[6] = tm_new.tm_min;
    txbuf[7] = tm_new.tm_sec;

    //clear events first
    rt_event_recv(&msg_event, EVENT_SYNC, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &e);

	CAN1_Send_bms(id, txbuf, 8);
    do
    {
        retry--;
//		LOG_D("bms sync can send data!\n");
        if (CAN1_Send_bms(id, txbuf, 8) < 0) {
            LOG_D("can send msg failed!\n");
            continue;
        }

        if (rt_event_recv(&msg_event, EVENT_SYNC, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, \
            MS_TO_TICKS(WAIT_MSG_TIMEOUT_SYNC), &e) != RT_EOK)
        {
            LOG_D("recv event timeout!\n");
            continue;
        } 
        else {
            //LOG_D("bms responses success!\n");
            break;
        }
    } while(retry);

    if (retry != 0) {
        //wait for the uploading done.
        rt_thread_mdelay(WAIT_MSG_TIMEOUT_SYNC);
        debug_info_raw_print();
        decode_data(1);
        ret = 0;
		retry_group = 0;
    }
    else {
		retry_group ++;
		if(retry_group > 4)	//5s
		{
			decode_data(0);
		}
    }
    debug_info_decoded_print();
    
    return ret;
}
/*
static struct rt_thread log_thread;
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t log_thread_stack[TASK_STACK_SIZE_BMSLOG];
static volatile uint8_t log_rd_task_running = 0;
static void task_getlog(void* _param);
int32_t bms_getlog(void)
{
    if (log_rd_task_running != 0) {
        LOG_W("it is getting bms logs!\n");
        return -1;
    }
    
    rt_err_t err = rt_thread_init(&log_thread, "bms.log", task_getlog, RT_NULL, &log_thread_stack[0],
                                   sizeof(log_thread_stack), TASK_PRIORITYBMSLOG, 20);
    if(err == RT_EOK) {
        rt_thread_startup(&log_thread);
        log_rd_task_running = 1;
        return 0;
    }
    return -1;
}

static void task_getlog(void* _param)
{
    uint8_t retry = 5;
    uint32_t e = 0;
    uint32_t id = 0;
    uint8_t txbuf[8] = {0};

    id = 0x1460A0B0;
    txbuf[0] = 0xE1;
    
    //clear events first
    rt_event_recv(&msg_event, EVENT_LOG | EVENT_PRELOG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &e);
    
    rt_memset(&BmsLog, 0, sizeof(BmsLog));
    
    do
    {
        retry--;
        if (CAN1_Send_bms(id, txbuf, 8) < 0) {
            LOG_D("can send msg failed!\n");
            continue;
        } else {
            break;
        }
    } while(retry);

    if (retry == 0) {
        LOG_D("get bms logs failed!\n");
        goto _exit;
    }
    
    if (rt_event_recv(&msg_event, EVENT_PRELOG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, \
                MS_TO_TICKS(WAIT_MSG_TIMEOUT_PRELOG), &e) != RT_EOK) {
        LOG_D("Not any response when get logs!\n");
        goto _exit;
    }

    if (BmsLog.tar_cnts == 0) {
        LOG_W("there is not any logs.\n");
        goto _exit;
    }
    
    while(1)
    {
        //if no logs upload in WAIT_MSG_TIMEOUT_LOG*2 MS, it indicate that the uploading is finish.
        if (rt_event_recv(&msg_event, EVENT_LOG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, \
                    MS_TO_TICKS(WAIT_MSG_TIMEOUT_LOG*2), &e) == RT_EOK) {
            continue;
        }
        else {
            LOG_W("get bms logs finish, cnts: %d\n", BmsLog.cnts);
            g_debug_print_log = 0;
            goto _exit;
        }
    }

_exit:
    log_rd_task_running = 0;
}
*/


int32_t bms_getversion(bms_ver_idx_t idx)
{
    uint8_t retry = 5;
    uint32_t e = 0;
    uint32_t id = 0;
    uint8_t txbuf[8] = {0xFF};
    uint32_t event = 0;

    id = 0x14E0A0B0;
    if (idx == E_BMS_VER_SOFT) {
        txbuf[0] = 0xA1;
        event = EVENT_VER_SOFT;
    }
    if (idx == E_BMS_VER_HARD) {
        txbuf[0] = 0xA2;
        event = EVENT_VER_HARD;
    }

    //clear events first
    rt_event_recv(&msg_event, event, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &e);

    do
    {
        retry--;
        if (CAN1_Send_bms(id, txbuf, 8) < 0) {
            LOG_D("can send msg failed!\n");
            continue;
        }

        if (rt_event_recv(&msg_event, event, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, \
            MS_TO_TICKS(WAIT_MSG_TIMEOUT_VER), &e) != RT_EOK)
        {
            LOG_D("recv event timeout!\n");
            continue;
        } 
        else {
            //LOG_D("bms responses success!\n");
            break;
        }
    } while(retry);

    if (retry != 0) {
        return 0;
    } 
    else {
        return -1;
    }
}


bms_data_t* bat_get_bms_data_ptr(void)
{
    return &BmsData;
}

uint8_t bms_get_wakeup_source(void)
{
	return BmsData.wakeup_source;
}

void get_bms_hw_version(char* _version)
{
	rt_memcpy(_version, BmsVersion.hard.version, sizeof(BmsVersion.hard.version));
}

void get_bms_sw_version(char* _version)
{
	rt_memcpy(_version, BmsVersion.soft.version, sizeof(BmsVersion.soft.version));
}

uint16_t get_bms_CellVolt(uint8_t _ch)
{
	return BmsData.CellVolt[_ch];
}

uint16_t get_bms_TotalVolt(void)
{
	return BmsData.TotalVolt;
}

uint16_t get_bms_TotalCurr(void)
{
	return BmsData.TotalCurr;
}

static void debug_info_decoded_print(void)
{
    if (g_debug_print_info == 0) return;
    
    bat_data_t* pdata = RT_NULL;
    pdata = bat_get_data_ptr();
    
    rt_kprintf("\r\n");
    rt_kprintf("[volt 0.1V]%d [curr 0.1A]%d [SOC]%d [cellvolt_low mV]%d [bat_status]%d \r\n", \
                pdata->volt_01V, pdata->curr_01A, pdata->soc, pdata->cellvolt_low_mV, pdata->status);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
}

static void debug_info_raw_print(void)
{
    bms_data_t* pbms_data = RT_NULL;
    pbms_data = &BmsData;
    
    if (g_debug_raw_print_info == 0) return;
    
    rt_kprintf("\r\n");
    rt_kprintf("\r\n");
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[TotalVolt]%d  [TotalCurr]%d  [SOC]%d [SOH]%d \n",  \
    pbms_data->TotalVolt, pbms_data->TotalCurr, pbms_data->SOC, pbms_data->SOH);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");

    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[PackVolt]%d [PackCap]%d \n",  \
    pbms_data->PackVolt, pbms_data->PackCap);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[CellVolt_High]%d  [CellVolt_Low]%d [PackNo_CellVoltHigh]%d [PackNo_CellVoltLow]%d \n",  \
    pbms_data->CellVolt_High, pbms_data->CellVolt_Low, pbms_data->PackNo_CellVoltHigh, pbms_data->PackNo_CellVoltLow);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[BunchNo_CellVoltHigh]%d [BunchNo_CellVoltLow]%d [CellTempHigh]%d [CellTempLow]%d \n",  \
    pbms_data->BunchNo_CellVoltHigh, pbms_data->BunchNo_CellVoltLow, pbms_data->CellTempHigh, pbms_data->CellTempLow);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[PackNo_CellTempHigh]%d [PackNo_CellTempLow]%d [BunchNo_CellTempHigh]%d [BunchNo_CellTempLow]%d\n",  \
    pbms_data->PackNo_CellTempHigh, pbms_data->PackNo_CellTempLow, pbms_data->BunchNo_CellTempHigh, pbms_data->BunchNo_CellTempLow);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");

    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[BmsLife]%d [ChgMosStatus]%d [DisChgMosStatus]%d [ExIoStatus]%d\n",  \
    pbms_data->BmsLife, pbms_data->ChgMosStatus, pbms_data->DisChgMosStatus, pbms_data->ExIoStatus);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[ChgDisChg_Cnts]%d [SleepFlag]%d [ChgStatus]%d\n",  \
    pbms_data->ChgDisChg_Cnts, pbms_data->SleepFlag, pbms_data->ChgStatus);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[CellVolt1]%d [CellVolt2]%d [CellVolt3]%d [CellVolt4]%d \n",  \
    pbms_data->CellVolt[0], pbms_data->CellVolt[1], pbms_data->CellVolt[2], pbms_data->CellVolt[3]);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");

    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[CellVolt5]%d [CellVolt6]%d [CellVolt7]%d [CellVolt8]%d \n",  \
    pbms_data->CellVolt[4], pbms_data->CellVolt[5], pbms_data->CellVolt[6], pbms_data->CellVolt[7]);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");

    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[CellVolt9]%d [CellVolt10]%d [CellVolt11]%d [CellVolt12]%d \n",  \
    pbms_data->CellVolt[8], pbms_data->CellVolt[9], pbms_data->CellVolt[10], pbms_data->CellVolt[11]);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");

    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[CellVolt13]%d [CellVolt14]%d [CellVolt15]%d [CellVolt16]%d \n",  \
    pbms_data->CellVolt[12], pbms_data->CellVolt[13], pbms_data->CellVolt[14], pbms_data->CellVolt[15]);
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[temp1~temp16]");
    for(uint8_t i = 0; i<sizeof(pbms_data->CellTemp); i++) {
        rt_kprintf("%d ", pbms_data->CellTemp[i]);
    }
    rt_kprintf("\r\n");
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");

    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
    rt_kprintf("[warning bytes]");
    for(uint8_t i = 0; i<8; i++) {
        rt_kprintf("0x%02X ", pbms_data->uWarn.Warn[i]);
    }
    rt_kprintf("\r\n");
    rt_kprintf("-----------------------------------------------------------------------------------------------\n");
}



//-----------------------------------------------------------------------------------------------------------------------------------------

static void bms(uint8_t argc, char **argv)
{
	const char* help_info[] =
    {
        [0]     = "bms -info/-raw <0/1>                   - print bms information",
        [1]     = "bms -ctrl <shutdown/sleep>             - control bms",
        [2]     = "bms -log                               - get bms log",
        [3]     = "bms -ver                               - get bms verson",
    };

	if (argc < 2)
    {
        rt_kprintf("Usage:\n");
        for (int i = 0; i < sizeof(help_info) / sizeof(char*); i++)
        {
            rt_kprintf("%s\n", help_info[i]);
        }
        rt_kprintf("\n");
    }
    else
    {
        const char *operator = argv[1];
        if (!rt_strcmp(operator, "-info")) 
        {
            if (argc < 3) {
                rt_kprintf("%s\n", help_info[0]);
            } 
            else
            {
                g_debug_print_info = atoi(argv[2]);
            }

        }
        else if (!rt_strcmp(operator, "-raw")) 
        {
            if (argc < 3) {
                rt_kprintf("%s\n", help_info[0]);
            } 
            else
            {
                g_debug_raw_print_info = atoi(argv[2]);
            }

        }
        else if (!rt_strcmp(operator, "-log")) 
        {
//            g_debug_print_log = 1;
//            bms_getlog();
        }
        else if (!rt_strcmp(operator, "-ver")) 
        {
            bms_getversion(E_BMS_VER_SOFT);
            bms_getversion(E_BMS_VER_HARD);
            rt_kprintf("bms softverson: %s\n", BmsVersion.soft.version);
            rt_kprintf("bms hardverson: %s\n", BmsVersion.hard.version);
        }
        else if (!rt_strcmp(operator, "-ctrl")) 
        {
            if (argc < 3) {
                rt_kprintf("%s\n", help_info[1]);
            } 
            else 
            {
                const char *cmd = argv[2];
                if (!rt_strcmp(cmd, "shutdown")) {
                    bms_control(BMS_CTRL_SHUTDOWN);
                }
                else if (!rt_strcmp(cmd, "sleep")) {
                    bms_control(BMS_CTRL_SLEEP);
                }
                else {
                    rt_kprintf("%s\n", help_info[1]);
                }
            }
        }
        else
        {
            rt_kprintf("Usage:\n");
            for (int i = 0; i < sizeof(help_info) / sizeof(char*); i++)
            {
                rt_kprintf("%s\n", help_info[i]);
            }
            rt_kprintf("\n");
        }
    }
}
MSH_CMD_EXPORT(bms, bms commands);

