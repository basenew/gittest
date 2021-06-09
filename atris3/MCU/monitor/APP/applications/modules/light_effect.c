/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : light_effect.c
 * Brief      : 灯效显示

 * Change Logs
 * Date           Author          Notes
 * 2019-11-24     wuxiaofeng      first version        
 */
#include "rtthread.h"
#include "rtdevice.h"
#include "common.h"
#include "board.h"
#include "app_cfg.h"
#include "light_effect.h"

#define LOG_TAG "light"
#define LOG_LVL LOG_LVL_INFO
#include <rtdbg.h>

#define DF_FOREVER 0xFFFFFFFF

#define DF_DEV1_PWM "pwm1"

#define DF_PWM_CH_RED    2
#define DF_PWM_CH_GREEN  1
#define DF_PWM_CH_BLUE   3

//#define DF_PWM_PERIOD (1000000) // 1ms 1KHZ

#define DF_PWM_PERIOD (10000000) //10ms 100HZ

#define DF_DO_TIMEROUT 10

#define DF_LUMI_MAX 100        
#define DF_LUMI_MIN_BREATHE 4

typedef int32_t (*dofun_t)(uint8_t *);

typedef enum
{
    R = 0,
    G,
    B,
} rgb_t;

typedef struct
{
    light_color_t color;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t lumi;
    uint8_t lumi_max;
} light_attr_t;

typedef struct
{
    light_event_t event;
    light_attr_t attr;
    dofun_t dofun;
    uint32_t maxcnt;
    uint32_t cnt;
} light_effect_t;

typedef struct
{
    light_event_t evt_running;
    uint32_t evt_bits;
    rt_mq_t mq;
} light_t;

typedef enum
{
    DOFUN_HOLD = 0,
    DOFUN_BREATHE,
    DOFUN_STROBE,
} action_t;

static int32_t do_hold(uint8_t *_lumi);
static int32_t do_breathe(uint8_t *_lumi);
static int32_t do_strobe(uint8_t *_lumi);

static rt_timer_t do_timer;
static struct rt_device_pwm *pwm_dev1 = RT_NULL;

static light_effect_t effect_table[LIGHT_EVT_MAX] = 
{
    {LIGHT_EVT_DISABLE, {C_IDLE, 0, 0, 0, 0, DF_LUMI_MAX}, do_hold, DF_FOREVER, 0},
    {LIGHT_EVT_BAT_CHARGE_ING, {C_GREEN, 0, 0, 0, DF_LUMI_MAX, DF_LUMI_MAX}, do_breathe, DF_FOREVER, 0},
    {LIGHT_EVT_BAT_CHARGE_FULL, {C_GREEN, 0, 0, 0, DF_LUMI_MAX, DF_LUMI_MAX}, do_hold, DF_FOREVER, 0},
    {LIGHT_EVT_WARN, {C_RED, 0, 0, 0, DF_LUMI_MAX, DF_LUMI_MAX}, do_hold, DF_FOREVER, 0},
    {LIGHT_EVT_CONTROL, {C_WHITE, 0, 0, 0, DF_LUMI_MAX, DF_LUMI_MAX}, do_hold, DF_FOREVER, 0},
    {LIGHT_EVT_STANDBY, {C_BLUE, 0, 0, 0, DF_LUMI_MAX, DF_LUMI_MAX}, do_breathe, DF_FOREVER, 0},
    {LIGHT_EVT_WORK, {C_BLUE, 0, 0, 0, DF_LUMI_MAX, DF_LUMI_MAX}, do_hold, DF_FOREVER, 0},
};

static volatile light_t light_obj;
//static mcu_data_light_t light_data;

static int32_t pwm_dev_init(void)
{
    rt_uint32_t pulse = 0;

    pwm_dev1 = (struct rt_device_pwm *)rt_device_find(DF_DEV1_PWM);
    if (pwm_dev1 == RT_NULL)
    {
        LOG_D("can't find %s device!\n", DF_DEV1_PWM);
        return -DF_ERR;
    }
    rt_pwm_set(pwm_dev1, DF_PWM_CH_RED, DF_PWM_PERIOD, pulse);
    rt_pwm_set(pwm_dev1, DF_PWM_CH_GREEN, DF_PWM_PERIOD, pulse);
    rt_pwm_set(pwm_dev1, DF_PWM_CH_BLUE, DF_PWM_PERIOD, pulse);
    rt_pwm_enable(pwm_dev1, DF_PWM_CH_RED);
    rt_pwm_enable(pwm_dev1, DF_PWM_CH_GREEN);
    rt_pwm_enable(pwm_dev1, DF_PWM_CH_BLUE);
    return DF_OK;
}

static int32_t hw_init(void)
{
    pwm_dev_init();
    return DF_OK;
}

//hold effect
static int32_t do_hold(uint8_t *_lumi)
{
    uint8_t result = DF_ING;
    static uint32_t s_tick = 0;

    if (s_tick >= 1000 / DF_DO_TIMEROUT)
    {
        s_tick = 0;
        result = DF_OK;
    }
    else
    {
        s_tick ++;
    }
    return result;
}

//breathe effect
static int32_t do_breathe(uint8_t *_lumi)
{
    static uint8_t s_state = 0;
    static uint32_t s_tick = 0;
    uint8_t result = DF_ING;
    static uint8_t s_lumi = DF_LUMI_MIN_BREATHE;

    switch (s_state)
    {
    case 0: // shutdown
        if (s_tick >= 50) //500 / DF_DO_TIMEROUT
        {
            s_state ++;
            s_tick = 0;
        }
        s_lumi = DF_LUMI_MIN_BREATHE;
        s_tick ++;
        break;

    case 1: // turning up
        s_tick ++;
        if (s_tick / 2)
        {
            //s_lumi += 2;
            s_lumi ++;
            s_tick = 0;
        }
        if (s_lumi >= DF_LUMI_MAX)
        {
            s_lumi = DF_LUMI_MAX;
            
            s_state ++;
            s_tick = 0;
        }
        break;

    case 2: // turning down
        s_tick ++;
        if (s_tick / 2)
        {
            //s_lumi -= 2;
            s_lumi --;
            s_tick = 0;
        }

        if (s_lumi <= DF_LUMI_MIN_BREATHE)
        {
            s_lumi = DF_LUMI_MIN_BREATHE;
            
            result = DF_OK;
            s_state = 0;
            s_tick = 0;
        }
        break;

    default:
        break;
    }

    *_lumi = s_lumi;
    return result;
}

//strobe effect
static int32_t do_strobe(uint8_t *_lumi)
{
    static uint32_t s_tick = 0;
    static uint8_t s_state = 0;
    uint8_t result = DF_ING;
    switch (s_state)
    {
    case 0: // shut down
        if (s_tick >= 120 / DF_DO_TIMEROUT)
        {
            s_state ++;
            s_tick = 0;
        }
        *_lumi = 0;
        s_tick ++;
        break;

    case 1: // light up
        if (s_tick >= 100 / DF_DO_TIMEROUT)
        {
            s_state ++;
            s_tick = 0;
        }
        *_lumi = 100;
        s_tick ++;
        break;

    case 2: // shut down
        if (s_tick > 120 / DF_DO_TIMEROUT)
        {
            s_state = 0;
            s_tick = 0;
            result = DF_OK;
        }
        *_lumi = 0;
        s_tick ++;
        break;

    default:
        break;
    }
    return result;
}

static void set_output(rgb_t _rgb, uint8_t _lumi)
{
    static uint32_t s_pulse_r = 0, s_pulse_g = 0, s_pulse_b = 0;
    uint32_t pulse = _lumi * DF_PWM_PERIOD / DF_LUMI_MAX;

    if (R == _rgb)
    {
        if (s_pulse_r != pulse)
        {
            s_pulse_r = pulse;
            rt_pwm_set(pwm_dev1, DF_PWM_CH_RED, DF_PWM_PERIOD, pulse);
        }
    }
    else if (G == _rgb)
    {
        if (s_pulse_g != pulse)
        {
            s_pulse_g = pulse;
            rt_pwm_set(pwm_dev1, DF_PWM_CH_GREEN, DF_PWM_PERIOD, pulse);
        }
    }
    else if (B == _rgb)
    {
        if (s_pulse_b != pulse)
        {
            s_pulse_b = pulse;
            rt_pwm_set(pwm_dev1, DF_PWM_CH_BLUE, DF_PWM_PERIOD, pulse);
        }
    }
    else
    {
        ;
    }
}

static void set_color(light_attr_t *_attr)
{
    uint8_t r, g, b;

    if (_attr == RT_NULL) return;
    
    if (_attr->lumi > _attr->lumi_max) {
        _attr->lumi = _attr->lumi_max;
    }
    
    switch (_attr->color)
    {
    case C_IDLE:
        /*
        g = _attr->r;
        g = _attr->g;
        b = _attr->b;
        */
        r = (uint8_t)(((uint32_t)_attr->r * (uint32_t)_attr->lumi) / (DF_LUMI_MAX));
        g = (uint8_t)(((uint32_t)_attr->g * (uint32_t)_attr->lumi) / (DF_LUMI_MAX));
        b = (uint8_t)(((uint32_t)_attr->b * (uint32_t)_attr->lumi) / (DF_LUMI_MAX));
        break;

    case C_RED:
        r = _attr->lumi;
        g = 0;
        b = 0;
        break;

    case C_YELLOW:
        r = _attr->lumi;
        g = _attr->lumi;
        b = 0;
        break;

    case C_GREEN:
        r = 0;
        g = _attr->lumi;
        b = 0;
        break;

    case C_CYAN:
        r = 0;
        g = _attr->lumi;
        b = _attr->lumi;
        break;

    case C_BLUE:
        r = 0;
        g = 0;
        b = _attr->lumi;
        break;

    case C_PURPLE:
        r = _attr->lumi;
        g = 0;
        b = _attr->lumi;
        break;

    case C_WHITE:
        r = _attr->lumi;
        g = _attr->lumi;
        b = _attr->lumi;
        break;

    case C_BLACK:
        r = 0;
        g = 0;
        b = 0;
        break;

    default:
        break;
    }

    set_output(R, r);
    set_output(G, g);
    set_output(B, b);
}

static void handle_effect(void)
{
    volatile light_t *obj = &light_obj;
    
    if (obj->evt_running >= LIGHT_EVT_MAX) {
        obj->evt_running = (light_event_t)(LIGHT_EVT_MAX - 1);
    }

    if (effect_table[obj->evt_running].maxcnt != DF_FOREVER)
    {
        if (effect_table[obj->evt_running].cnt >= effect_table[obj->evt_running].maxcnt)
        {
            effect_table[obj->evt_running].cnt = 0;
            light_post_event(obj->evt_running, 0);
        }
        else
        {
            ;
        }
    }
    //refresh lumi
    if (effect_table[obj->evt_running].dofun != RT_NULL)
    {
        if (effect_table[obj->evt_running].dofun(&(effect_table[obj->evt_running].attr.lumi)) == DF_OK)
        {
            effect_table[obj->evt_running].cnt++; // just count anyway
        }
    }
    //refresh color 
    set_color(&(effect_table[obj->evt_running].attr));
}

static void do_timeout(void *_param)
{
    handle_effect();
}

int32_t light_post_event(light_event_t _evt, int32_t _status)
{
    uint8_t cur_status = 0;
    volatile light_t* pobj = &light_obj;
    
    if (light_obj.mq == RT_NULL) return -DF_ERR;
    
    //get the event status
    cur_status = (pobj->evt_bits >> (uint8_t)_evt) & 0x1;
    if (cur_status != _status)
    {//update event
        light_msg_t msg;
        msg.evt = _evt;
        msg.buffer[0] = _status;
        return (int32_t)rt_mq_send(light_obj.mq, &msg, sizeof(light_msg_t));
    }
    return DF_OK;
}


//static void data_report(void)
//{
//    light_data.event = light_obj.evt_running;
//    remote_notify_msg(MCU_DATA_ID2_LIGHT, (mcu_data_t*)&light_data, sizeof(mcu_data_light_t));
//}

static void task_main(void *_param)
{
    uint32_t s_timer1 = 0;
    light_msg_t msg;
    light_event_t evt_running_bk = LIGHT_EVT_DISABLE;
    
    while (1)
    {
        if (rt_mq_recv(light_obj.mq, &msg, sizeof(light_msg_t), rt_tick_from_millisecond(100)) == RT_EOK)
        {
            //control the priority of events.
            if (msg.buffer[0] != 0)
            {
                light_obj.evt_bits |= (1 << msg.evt);
            }
            else
            {
                light_obj.evt_bits &= ~(1 << msg.evt);
            }

            light_obj.evt_running = (light_event_t)find_first_1bit_uint32(light_obj.evt_bits, LIGHT_EVT_MAX);
            if(light_obj.evt_running != evt_running_bk)
            {
                if (light_obj.evt_running >= LIGHT_EVT_MAX)
                {
                    light_obj.evt_running = (light_event_t)(LIGHT_EVT_MAX - 1);
                    effect_table[light_obj.evt_running].attr.color = C_BLACK;
                    set_color(&(effect_table[light_obj.evt_running].attr));
                    rt_timer_stop(do_timer);
                }
                else
                {
                    if (!(do_timer->parent.flag & RT_TIMER_FLAG_ACTIVATED))
                    { //if the timer is not running.
                        rt_timer_start(do_timer);
                    }
                } 
                evt_running_bk = light_obj.evt_running;
                LOG_W("light event running: %d", light_obj.evt_running);
            }
        }

        if(os_gettime_ms() - s_timer1 >= 500)
        {
            s_timer1 = os_gettime_ms();
//            data_report();
        } else if (os_gettime_ms() < s_timer1) {
            s_timer1 = os_gettime_ms();
        }
    }
}

int32_t light_effect_init(void)
{
    hw_init();

    do_timer = rt_timer_create("light",
                               do_timeout,
                               RT_NULL,
                               rt_tick_from_millisecond(DF_DO_TIMEROUT),
                               RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    if (do_timer == RT_NULL)
    {
        return -RT_ERROR;
    }

    light_obj.mq = rt_mq_create("light.mq", sizeof(light_msg_t), 8, RT_IPC_FLAG_FIFO);
    if (light_obj.mq == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_thread_t thread = rt_thread_create("light",
                                          task_main,
                                          RT_NULL,
                                          TASK_STACK_SIZE_LIGHT,
                                          TASK_PRIORITY_LIGHT,
                                          20);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        rt_timer_delete(do_timer);
        rt_mq_delete(light_obj.mq);
        return -RT_ERROR;
    }
    
//    remote_service_add(MCU_CMD_ID2_LIGHT, light_msg_remote_cb, DF_YES);
    
    //default
    light_post_event(LIGHT_EVT_WORK, 1);
    
    return RT_EOK;
}


int32_t light_remote_set(light_status_t light_status)
{
	if (light_status.action == DOFUN_HOLD)
	{
		effect_table[LIGHT_EVT_CONTROL].dofun = do_hold;
	}
	else if (light_status.action == DOFUN_BREATHE)
	{
		effect_table[LIGHT_EVT_CONTROL].dofun = do_breathe;
	}
	else if (light_status.action == DOFUN_STROBE)
	{
		effect_table[LIGHT_EVT_CONTROL].dofun = do_strobe;
	}
	else
	{
		;
	}

	effect_table[LIGHT_EVT_CONTROL].attr.color = light_status.color;
	effect_table[LIGHT_EVT_CONTROL].attr.lumi_max = light_status.lum_max;
	effect_table[LIGHT_EVT_CONTROL].maxcnt = DF_FOREVER;
	effect_table[LIGHT_EVT_CONTROL].cnt = 0; 
	light_post_event(LIGHT_EVT_CONTROL, light_status.ctr_right);
	
	return 0;

}

light_status_t light_current_status;
light_status_t* light_remote_get(void)
{
	light_status_t* ptr = &light_current_status;
	light_event_t light_current_event;
	
	
	light_current_event = light_obj.evt_running;
	if (effect_table[light_current_event].dofun == do_hold)
	{
		light_current_status.action = DOFUN_HOLD;
	}
	else if (effect_table[light_current_event].dofun == do_breathe)
	{
		light_current_status.action = DOFUN_BREATHE;
	}
	else if (effect_table[light_current_event].dofun == do_strobe)
	{
		light_current_status.action = DOFUN_STROBE;
	}
	else
	{
		;
	}
	light_current_status.color = effect_table[light_current_event].attr.color;
	light_current_status.lum_max = effect_table[light_current_event].attr.lumi_max;
	
	return ptr;

}

uint32_t light_get_evt_bits(void)
{
	return light_obj.evt_bits;
}
//---------------------------------------------------------------------------------------------------------------------

static void light(uint8_t argc, char **argv)
{
    if (argc != 3)
    {
        rt_kprintf("Please input: light <evtID> <1/0>\n");
    }
    else
    {
        light_event_t event = (light_event_t)atoi(argv[1]);
        uint8_t status = atoi(argv[2]);

        //light_obj.evt_bits = 0; // clear all events first
        light_post_event(event, status);
    }
}
MSH_CMD_EXPORT(light, light effect);


static void light_evt(uint8_t argc, char **argv)
{
    if (argc != 1)
    {
        rt_kprintf("Please input: light_evt\n");
    }
    else
    {
        rt_kprintf("light event id: 0x%02X\n", light_obj.evt_running);
    }
}
MSH_CMD_EXPORT(light_evt, light event id);


static void light_test(uint8_t argc, char **argv)
{
	light_status_t light_status_test;
	light_status_t* _ptr;
	
    if (argc < 1)
    {
        rt_kprintf("Please input: light_test\n");
    }
    else
    {
		if(atoi(argv[1]) == 1)
		{
			light_status_test.action = atoi(argv[2]);
			light_status_test.color = (light_color_t)atoi(argv[3]);
			light_status_test.lum_max = atoi(argv[4]);
			light_status_test.ctr_right = atoi(argv[5]);
			light_remote_set(light_status_test);
		}
		else if(atoi(argv[1]) == 2)
		{
			_ptr = light_remote_get();
			
			rt_kprintf("light sta: act-%d, color-%d, lum-%d\n", _ptr->action, _ptr->color, _ptr->lum_max);
		}
    }
}
MSH_CMD_EXPORT(light_test, light test);




