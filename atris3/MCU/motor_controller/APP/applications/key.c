/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : key.c
 * Brief      : 按键服务

 * Change Logs
 * Date           Author          Notes
 * 2019-11-24     wuxiaofeng      first version        
 */

#include "board.h"
#include "sw.h"
#include "app_cfg.h"
#include "common.h"
#include "key.h"

#define LOG_TAG "key"
#define LOG_LVL LOG_LVL_DBG
#include <rtdbg.h>

#ifndef TASK_STACK_SIZE_KEY
#define TASK_STACK_SIZE_KEY 1024
#endif

#ifndef TASK_PRIORITY_KEY
#define TASK_PRIORITY_KEY 12
#endif


#define KEY_SOFT_BUTTUN_LONGPRESS_MS 1500

#define KEY_SUBSCRIBER_NUM_MAX 8

typedef struct
{
    rt_mq_t mq;
    uint32_t val;
    uint8_t subscriber_num;
    key_subscriber_t subscriber[KEY_SUBSCRIBER_NUM_MAX];
} ukey_t;

typedef struct
{
    sw_idx_t idx;
    uint8_t status;
} sw_msg_t;

static ukey_t key_obj = {0};

static key_val_c_t val_c[KEY_VAL_MAX] = {
    {KEY_VAL_INVALID, "invalid"},
    {KEY_VAL_SFOTKEY_IN, "softkey_in"},
    {KEY_VAL_SFOTKEY_SHORT, "softkey_short"},
    {KEY_VAL_SFOTKEY_LONG, "softkey_long"},
    {KEY_VAL_SFOTKEY_KEEPLONG, "softkey_keeplong"},

};

static void key_publish(key_val_t _val)
{
    if (key_obj.subscriber_num == 0)
    {
        LOG_W("key not any subscriber.");
        return;
    }
    for (uint8_t i = 0; i < key_obj.subscriber_num; ++i)
    {
        if (key_obj.subscriber[i] != RT_NULL)
            key_obj.subscriber[i](_val);
    }
}

void key_subscribe(key_subscriber_t _sub)
{
    rt_base_t level;
    level = rt_hw_interrupt_disable();
    if (key_obj.subscriber_num < KEY_SUBSCRIBER_NUM_MAX)
    {
        key_obj.subscriber[key_obj.subscriber_num++] = _sub;
        rt_hw_interrupt_enable(level);
    }
    else
    {
        rt_hw_interrupt_enable(level);
        LOG_W("key too many subscribers.");
    }
}

static void sw_callback(sw_idx_t _idx, uint8_t _status)
{
    sw_msg_t msg;
    msg.idx = _idx;
    msg.status = _status;
    rt_mq_send(key_obj.mq, &msg, sizeof(sw_msg_t));
}

static void task_main(void *_param)
{
    sw_msg_t msg;
    uint32_t flesh_val = 0;
    uint8_t pin_status = PIN_LOW;
    uint32_t s_timer1 = 0;

    while (1)
    {
        if (rt_mq_recv(key_obj.mq, &msg, sizeof(sw_msg_t), rt_tick_from_millisecond(100)) == RT_EOK)
        {
            switch (msg.idx)
            {
            case SW_SOFT_BUTTUN:
            {
                pin_status = sw_unit_read(SW_SOFT_BUTTUN);
                if (pin_status == PIN_LOW)
                {
                    s_timer1 = os_gettime_ms();
                    flesh_val = KEY_VAL_SFOTKEY_IN;
                }
                else
                {
                    if ((s_timer1 != 0) && (os_gettime_ms() - s_timer1 >= KEY_SOFT_BUTTUN_LONGPRESS_MS))
                    {
                        flesh_val = KEY_VAL_SFOTKEY_LONG;
                    }
                    else
                    {
                        flesh_val = KEY_VAL_SFOTKEY_SHORT;
                    }
                }
            }
            break;

            default:
                break;
            }
        }

        if (flesh_val == KEY_VAL_SFOTKEY_IN)
        {
            if ((s_timer1 != 0) && (os_gettime_ms() - s_timer1 >= KEY_SOFT_BUTTUN_LONGPRESS_MS))
            {
                flesh_val = KEY_VAL_SFOTKEY_KEEPLONG;
                s_timer1 = 0;
            }
        }

        if (flesh_val != key_obj.val && flesh_val != KEY_VAL_INVALID)
        {
            key_obj.val = flesh_val;
            LOG_W("key value is %s", val_c[flesh_val].name);
            key_publish((key_val_t)key_obj.val);
        }
    }
}

int32_t key_init(void)
{
    sw_unit_attach(SW_SOFT_BUTTUN, sw_callback);

    sw_unit_enable(SW_SOFT_BUTTUN, SW_POLL_SLICE);


    key_obj.mq = rt_mq_create("key.mq", sizeof(sw_msg_t), 8, RT_IPC_FLAG_FIFO);
    if (key_obj.mq == RT_NULL)
        return -RT_ERROR;

    rt_thread_t thread = rt_thread_create("key",
                                          task_main,
                                          RT_NULL,
                                          TASK_STACK_SIZE_KEY,
                                          TASK_PRIORITY_KEY,
                                          20);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        rt_mq_delete(key_obj.mq);
        return -RT_ERROR;
    }
    return RT_EOK;
}
