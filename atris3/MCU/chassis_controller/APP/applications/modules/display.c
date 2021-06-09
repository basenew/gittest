/*
 * COPYRIGHT (C) Copyright 2018-2028; UBT TECH; SHENZHEN, CHINA
 *
 * File       : display.c
 * Brief      : 显示屏驱动

 * Change Logs
 * Date           Author        Version       Notes
 * 2018-12-18     wuxiaofeng    v1.0
 *
 *
 */

#include "common.h"
#include "app_cfg.h"
#include "battery.h"
#include "display.h"
#include "rtdevice.h"

#define LOG_TAG              "display"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>


#define DISPLAY_DATA_HEAD     0xC5

#define LED_F             0x000001
#define LED_A             0x000002
#define LED_C             0x000004
#define LED_H             0x000008
#define LED_V             0x000010
#define LED_PERCENT       0x000020
#define LED_BAT1          0x000040
#define LED_BAT2          0x000080
#define LED_BAT3          0x000100
#define LED_BAT4          0x000200
#define LED_BAT5          0x000400
#define LED_BAT6          0x000800
#define LED_BAT7          0x001000
#define LED_BAT8          0x002000
#define LED_BAT_FRAME     0x004000
#define LED_BAT_DP1       0x008000
#define LED_BAT_DP2       0x010000
#define LED_BAT_DP3       0x020000

#define CMD_DISPLAY_SWITCH    0x01
#define CMD_DISPLAY_DATA      0x02
#define CMD_DISPLAY_FLAG      0x03


/* Default config for serial_configure structure */
#define RT_SERIAL_CONFIG_DISPLAY           \
{                                          \
    19200 ,           /* 19200 bits/s */  \
    DATA_BITS_8,      /* 8 databits */     \
    STOP_BITS_1,      /* 1 stopbit */      \
    PARITY_NONE,      /* No parity  */     \
    BIT_ORDER_LSB,    /* LSB first sent */ \
    NRZ_NORMAL,       /* Normal mode */    \
    RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
    0                                      \
}


typedef struct
{
    char*        puart_name;
    rt_device_t  pdevice;

} display_t;


static display_t g_display =
{
    .puart_name     = "uart4",
    .pdevice        = RT_NULL,

};

static display_state_t g_display_state = DISPALY_NORMAL;



static int32_t display_device_init(display_t* _dev)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DISPLAY;
	
    if (_dev == RT_NULL)  return -1;
    if (_dev->puart_name == RT_NULL) return -1;

    //uart 设备
    rt_device_t uart_dev = rt_device_find(_dev->puart_name);
	rt_device_control(uart_dev, RT_DEVICE_CTRL_CONFIG, &config);
    if (uart_dev != RT_NULL && rt_device_open(uart_dev, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
    {
        if (_dev->pdevice != RT_NULL)
        {
            rt_device_close(_dev->pdevice);
        }
        _dev->pdevice = uart_dev;
        return 0;
    }
    LOG_E("display: fail to open device: %s\n", _dev->puart_name);
    return -1;
}


/**
 * 电池电量显示格
 * @param   _soc   : 电量
 * @return         :
 */
uint32_t display_batteryStep(uint8_t _soc)
{
    static uint8_t s_battery_charge_step = 0;
    uint32_t       battery_flag_data = 0;
    bat_data_t* pdata = RT_NULL;
    

    pdata = bat_get_data_ptr();

    uint8_t battery_step = (_soc / 13) + 1;
    if (battery_step > 8) {
        battery_step = 8;
    }

    if (pdata->status == BAT_STATUS_CHARGING)
    {
        s_battery_charge_step ++;
        if (s_battery_charge_step > battery_step)
        {
            s_battery_charge_step = 0;
        }
        battery_step = s_battery_charge_step;
    }

    switch (battery_step)
    {
    case 1:
        battery_flag_data = LED_BAT_FRAME
                            | LED_BAT1;
        break;

    case 2:
        battery_flag_data = LED_BAT_FRAME
                            | LED_BAT1
                            | LED_BAT2;
        break;

    case 3:
        battery_flag_data = LED_BAT_FRAME
                            | LED_BAT1
                            | LED_BAT2
                            | LED_BAT3;
        break;

    case 4:
        battery_flag_data = LED_BAT_FRAME
                            | LED_BAT1
                            | LED_BAT2
                            | LED_BAT3
                            | LED_BAT4;
        break;

    case 5:
        battery_flag_data = LED_BAT_FRAME
                            | LED_BAT1
                            | LED_BAT2
                            | LED_BAT3
                            | LED_BAT4
                            | LED_BAT5;
        break;

    case 6:
        battery_flag_data = LED_BAT_FRAME
                            | LED_BAT1
                            | LED_BAT2
                            | LED_BAT3
                            | LED_BAT4
                            | LED_BAT5
                            | LED_BAT6;
        break;

    case 7:
        battery_flag_data = LED_BAT_FRAME
                            | LED_BAT1
                            | LED_BAT2
                            | LED_BAT3
                            | LED_BAT4
                            | LED_BAT5
                            | LED_BAT6
                            | LED_BAT7;
        break;

    case 8:
        battery_flag_data = LED_BAT_FRAME
                            | LED_BAT1
                            | LED_BAT2
                            | LED_BAT3
                            | LED_BAT4
                            | LED_BAT5
                            | LED_BAT6
                            | LED_BAT7
                            | LED_BAT8;
        break;

    default:
        battery_flag_data = LED_BAT_FRAME;
        break;
    }

    return battery_flag_data;
}

/**
 * 发送命令
 * @param     :
 * @return    :
 */
static inline uint32_t display_cmd_write(const uint8_t* _pbuffer, uint32_t _txlen)
{
    return rt_device_write(g_display.pdevice, 0, _pbuffer, _txlen);
}

/**
 * 显示屏开关设置
 * @param   _en    : 开关
 * @return         :
 */
static void display_switch_en(uint8_t _en)
{
    uint8_t txbuf[4]  = {0};
    uint8_t sum = 0;

    txbuf[0] = DISPLAY_DATA_HEAD;
    txbuf[1] = CMD_DISPLAY_SWITCH;
    txbuf[2] = _en;

    sum = 0;
    for (uint8_t i = 0; i < 2; i++)
    {
        sum += txbuf[i + 1];
    }
    txbuf[3] = sum;

    display_cmd_write(txbuf, sizeof(txbuf));
}

/**
 * 电量显示
 * @param       :
 * @return      :
 */
static void display_show_soc(void)
{
    uint8_t data[3]  = {0};
    uint8_t txbuf[6] = {0};
    uint8_t sum = 0, i = 0;
    bat_data_t* pdata = RT_NULL;
    

    display_switch_en(1);

    pdata = bat_get_data_ptr();
    uint8_t soc = pdata->soc ;
	
    if (soc > 100)
    {
        data[0] = 8;
        data[1] = 8;
        data[2] = 8;
    }
    else
    {
        data[0] = soc / 100;
        data[1] = (soc % 100) / 10;
        data[2] = soc % 10;
    }

    //数字显示
    txbuf[0] = DISPLAY_DATA_HEAD;
    txbuf[1] = CMD_DISPLAY_DATA;
    txbuf[2] = data[0];
    txbuf[3] = data[1];
    txbuf[4] = data[2];

    //校验和
    sum = 0;
    for (i = 0; i < 4; i++)
    {
        sum += txbuf[i + 1];
    }
    txbuf[5] = sum ;
    display_cmd_write(txbuf, sizeof(txbuf));

    //标号显示
    uint32_t flag_tmp = display_batteryStep(soc);
    flag_tmp |= LED_PERCENT;
    //flag_tmp = 0x3ffff;

    txbuf[0] = DISPLAY_DATA_HEAD;
    txbuf[1] = CMD_DISPLAY_FLAG;
    txbuf[2] = GETLLBYTE(flag_tmp);
    txbuf[3] = GETLHBYTE(flag_tmp);
    txbuf[4] = GETHLBYTE(flag_tmp);

    sum = 0;
    for (i = 0; i < 4; i++)
    {
        sum += txbuf[i + 1];
    }
    txbuf[5] = sum;
    display_cmd_write(txbuf, sizeof(txbuf));
}

/**
 * 电压显示
 * @param       :
 * @return      :
 */
static void display_show_voltage(void)
{
    uint8_t data[3]  = {0};
    uint8_t txbuf[6] = {0};
    uint8_t sum = 0, i = 0;
    bat_data_t* pdata = RT_NULL;
    

    display_switch_en(1);

    pdata = bat_get_data_ptr();
    uint8_t soc = pdata->soc;
    uint16_t display_voltage = pdata->volt_01V;
	
    if (display_voltage > 600)
    {
        data[0] = 8;
        data[1] = 8;
        data[2] = 8;
    }
    else
    {
        data[0] = display_voltage / 100;
        data[1] = (display_voltage % 100) / 10;
        data[2] = display_voltage % 10;
    }

    //数字显示
    txbuf[0] = DISPLAY_DATA_HEAD;
    txbuf[1] = CMD_DISPLAY_DATA;
    txbuf[2] = data[0];
    txbuf[3] = data[1];
    txbuf[4] = data[2];

    //校验和
    sum = 0;
    for (i = 0; i < 4; i++)
    {
        sum += txbuf[i + 1];
    }
    txbuf[5] = sum ;
    display_cmd_write(txbuf, sizeof(txbuf));

    //标号显示
    uint32_t flag_tmp = display_batteryStep(soc);
    flag_tmp |= LED_V | LED_BAT_DP2;
    //flag_tmp = 0x3ffff;

    txbuf[0] = DISPLAY_DATA_HEAD;
    txbuf[1] = CMD_DISPLAY_FLAG;
    txbuf[2] = GETLLBYTE(flag_tmp);
    txbuf[3] = GETLHBYTE(flag_tmp);
    txbuf[4] = GETHLBYTE(flag_tmp);

    sum = 0;
    for (i = 0; i < 4; i++)
    {
        sum += txbuf[i + 1];
    }
    txbuf[5] = sum;
    display_cmd_write(txbuf, sizeof(txbuf));
}

static void display_state_normal(void)
{
    static uint8_t display_normal_flag = 0;
    static uint8_t display_normal_cnt = 0;

//    if (bat_get_inf_flag() == NO)
//    {
//        return;
//    }

    display_normal_cnt ++;
    if (display_normal_cnt >= 10)
    {
        display_normal_cnt = 0;

        if (display_normal_flag)
        {
            display_normal_flag = 0;
        }
        else
        {
            display_normal_flag = 1;
        }
    }

    if (display_normal_flag)
    {
        display_show_soc();
    }
    else
    {
        display_show_voltage();
    }
}

static void display_state_testing(void)
{
    uint8_t data[3]  = {0};
    uint8_t txbuf[6] = {0};
    uint8_t sum = 0;
    uint8_t i = 0;
    uint32_t flag_tmp = 0;

    display_switch_en(1);

    data[0] = 8;
    data[1] = 8;
    data[2] = 8;

    //数字显示
    txbuf[0] = DISPLAY_DATA_HEAD;
    txbuf[1] = CMD_DISPLAY_DATA;
    txbuf[2] = data[0];
    txbuf[3] = data[1];
    txbuf[4] = data[2];

    //校验和
    sum = 0;
    for (i = 0; i < 4; i++)
    {
        sum += txbuf[i + 1];
    }
    txbuf[5] = sum ;
    display_cmd_write(txbuf, sizeof(txbuf));

    //标号显示
    flag_tmp = 0x3ffff;

    txbuf[0] = DISPLAY_DATA_HEAD;
    txbuf[1] = CMD_DISPLAY_FLAG;
    txbuf[2] = GETLLBYTE(flag_tmp);
    txbuf[3] = GETLHBYTE(flag_tmp);
    txbuf[4] = GETHLBYTE(flag_tmp);

    sum = 0;
    for (i = 0; i < 4; i++)
    {
        sum += txbuf[i + 1];
    }
    txbuf[5] = sum;
    display_cmd_write(txbuf, sizeof(txbuf));
}



/**
 * 设置显示屏状态
 * @param      : 
 * @return     : void
 */
void display_state_write(display_state_t _state)
{// no need datalock
    g_display_state = _state;
}

/**
 * 读取显示屏状态
 * @param      : 
 * @return     : void
 */
display_state_t display_state_read(void)
{// no need datalock
    return g_display_state;
}



void display_loop(void)
{    
    switch (g_display_state)
    {
    case DISPALY_NORMAL:
        display_state_normal();
        break;

    case DISPALY_WARNNIG:
        break;

    case DISPALY_TESTING:
        display_state_testing();
        break;

    default:
        break;
    }
}


int32_t display_init(void)
{
    display_device_init(&g_display);
    

    return 0;
}





//------------------------------------------------------------------------------------------------------------------





