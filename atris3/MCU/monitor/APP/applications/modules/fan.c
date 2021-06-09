/*
 * COPYRIGHT (C) Copyright 2012-2017; UBT TECH; SHENZHEN, CHINA
 *
 * File       : fan.c
 * Brief      : fan speed manager

 * Change Logs
 * Date           Author        Version       Notes
 * XX             licaixia      v1.0
 * 2018-01-11     wuxiaofeng    v1.1          RTT
 * 2020-08-14     wuxiaofeng    v2.0          
 */
#include "rtthread.h"
#include "board.h"
#include "common.h"
#include "app_cfg.h"
#include "fan.h"

#define LOG_TAG              "fan"
#define LOG_LVL               LOG_LVL_INFO
#include <ulog.h>

#define DF_DEV_PWM "pwm4"
#define DF_PWM_CH_FAN1   1
#define DF_PWM_CH_FAN2   2
#define DF_PWM_PERIOD  (50000) //50000ns, 0.05ms, 20KHZ  

#define FAN_FG1_PIN GET_PIN(G, 2)
#define FAN_FG2_PIN GET_PIN(G, 3)
#define FAN_FG3_PIN GET_PIN(G, 4)
#define FAN_FG4_PIN GET_PIN(G, 5)

#define FAN_DUTY_SPEED0   100
#define FAN_DUTY_SPEED1   70
#define FAN_DUTY_SPEED2   45
#define FAN_DUTY_SPEED3   0

typedef struct
{
    uint8_t     is_init;   //是否已经初始化
    uint8_t     run_flag;  //风扇是否在运行
    uint8_t     cur_speed; //当前风速
    uint8_t     rpm;
    uint8_t     test_flag; //是否处于测试状态
    uint8_t     err_info;
    rt_mq_t     mq;
} fan_t;

static fan_t  g_fan;
static struct rt_device_pwm *pwm_dev = RT_NULL;

//static void check_cb(void *args);



static int32_t pwm_dev_init(void)
{
    rt_uint32_t pulse = DF_PWM_PERIOD;

    pwm_dev = (struct rt_device_pwm *)rt_device_find(DF_DEV_PWM);
    if (pwm_dev == RT_NULL)
    {
        LOG_D("can't find %s device!\n", DF_DEV_PWM);
        return -DF_ERR;
    }
    rt_pwm_set(pwm_dev, DF_PWM_CH_FAN1, DF_PWM_PERIOD, pulse);
    rt_pwm_set(pwm_dev, DF_PWM_CH_FAN2, DF_PWM_PERIOD, pulse);
    
    rt_pwm_enable(pwm_dev, DF_PWM_CH_FAN1);
    rt_pwm_enable(pwm_dev, DF_PWM_CH_FAN2);

    return DF_OK;
}

static void fan_pwm_set(uint16_t duty)
{
    rt_uint32_t pulse = 0;
    pulse = DF_PWM_PERIOD / 100 * duty;
    rt_pwm_set(pwm_dev, DF_PWM_CH_FAN1, DF_PWM_PERIOD, pulse);
    rt_pwm_set(pwm_dev, DF_PWM_CH_FAN2, DF_PWM_PERIOD, pulse);
}

static void fan_set_speed(void)
{
    static uint8_t s_speed_pre = FAN_SPEED_LVL0;
           uint8_t speed_cur   = g_fan.cur_speed;

    if (speed_cur == s_speed_pre) {
        return;
    }

    if (speed_cur == FAN_SPEED_LVL0) {
        //FAN_PWREN_DISABLE;
    } 
    else {
        //FAN_PWREN_ENABLE;
        if (speed_cur >= FAN_SPEED_LVLMAX) {
            speed_cur = FAN_SPEED_LVLMAX - 1;
        }
    }
    
    switch(speed_cur)
    {
      case FAN_SPEED_LVL0:
        fan_pwm_set(FAN_DUTY_SPEED0);
		POWER_24V_FAN_DISABLE;
        break;
      case FAN_SPEED_LVL1:
        fan_pwm_set(FAN_DUTY_SPEED1);
		POWER_24V_FAN_ENABLE;
        break;
      case FAN_SPEED_LVL2:
        fan_pwm_set(FAN_DUTY_SPEED2);
		POWER_24V_FAN_ENABLE;
        break;
      case FAN_SPEED_LVL3:
        fan_pwm_set(FAN_DUTY_SPEED3);
		POWER_24V_FAN_ENABLE;
        break;
       default:
        break;
    }
    s_speed_pre = speed_cur;
    LOG_D("fan level change: %d", speed_cur);
}

/*
static TIM_HandleTypeDef htim6;
static volatile uint32_t gFanFgPeriod = 0;
static volatile uint32_t gFanRpm = 0;
static volatile uint32_t gFanMeasureFlag = 0;
#define TIMER6_PERIOD 60000
#define TIMER6_PRESCALER 840  //APB1*2 = 84000000
#define TIMER6_PERIOD_MS (0.01)

static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = TIMER6_PRESCALER - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = TIMER6_PERIOD - 1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

static int32_t measure_init(void)
{
    rt_pin_mode(FAN_FG_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(FAN_FG_PIN, PIN_IRQ_MODE_FALLING, check_cb, RT_NULL);
    MX_TIM6_Init();
    return 0;
}

static void check_cb(void *args)
{
    static uint8_t s_first_in = 0;
    static uint32_t a_cnts = 0;
    static uint32_t b_cnts = 0;
    
    if (s_first_in == 0) {
        a_cnts = htim6.Instance->CNT;
        s_first_in = 1;
    }
    else if (s_first_in == 1) {
        b_cnts = htim6.Instance->CNT;
        if (b_cnts >= a_cnts) {
            gFanFgPeriod = b_cnts - a_cnts;
        }
        else {
            gFanFgPeriod = b_cnts + TIMER6_PERIOD - a_cnts;
        }
        s_first_in = 0;
        rt_pin_irq_enable(FAN_FG_PIN, PIN_IRQ_DISABLE);
        HAL_TIM_Base_Stop(&htim6);
        gFanMeasureFlag = 2;
    }
}

static int32_t check_speed(void)
{
    int32_t ret = DF_ING;
    double T1 = 0.0;
    static uint8_t s_circle = 0;
    switch(gFanMeasureFlag)
    {
        case 0:
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Start(&htim6);
            rt_pin_irq_enable(FAN_FG_PIN, PIN_IRQ_ENABLE);
            gFanMeasureFlag = 1;
        break;
        
        case 1:
            s_circle ++;
            if (s_circle >= 2) {
                gFanFgPeriod = 0;
                gFanMeasureFlag = 2;
            }
        break;
        
        case 2:
            
            if (gFanFgPeriod == 0) {
                gFanRpm = 0;
            }
            else {
                T1 = (double)gFanFgPeriod / 2.0 * TIMER6_PERIOD_MS;
                gFanRpm = (uint32_t)(60.0*1000.0 / 4.0 / T1);
                
            }
//            LOG_D("gFanFgPeriod: %d\n", gFanFgPeriod);
//            LOG_D("gFanRpm: %d\n", gFanRpm);
            gFanMeasureFlag = 0;
            s_circle = 0;
            g_fan.rpm = gFanRpm;
            ret = DF_OK;
        break;
    
        default: break;
    }
    return ret;
}
*/
void fan_gpio_init(void)
{

    rt_pin_mode(PIN_POWER_24V_FAN_PD14,       PIN_MODE_OUTPUT);
//    rt_pin_mode(PIN_FAN_PWM_F_PD12,           PIN_MODE_OUTPUT);
//    rt_pin_mode(PIN_FAN_PWM_R_PD13,           PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_FAN_RD1_IN_PG2,           PIN_MODE_INPUT);
    rt_pin_mode(PIN_FAN_RD2_IN_PG3,           PIN_MODE_INPUT);
    rt_pin_mode(PIN_FAN_RD3_IN_PG4,           PIN_MODE_INPUT);
    rt_pin_mode(PIN_FAN_RD4_IN_PG5,           PIN_MODE_INPUT);
	
	POWER_24V_FAN_DISABLE;
}

static int32_t hw_init(void)
{
	fan_gpio_init();
    pwm_dev_init();
    //measure_init();
    return 0;
}

int32_t fan_mq_send(fan_msg_t * _msg)
{
    return rt_mq_send(g_fan.mq, _msg, sizeof(fan_msg_t));
}

static int32_t fan_mq_recv(fan_msg_t *_msg, uint32_t _timeout_ms)
{
    return rt_mq_recv(g_fan.mq, _msg, sizeof(fan_msg_t), rt_tick_from_millisecond(_timeout_ms));
}

//static int32_t fan_msg_remote_cb(const mcu_cmd_t *_msg, mcu_cmd_resp_t *_resp)
//{
//    if (g_fan.mq == RT_NULL) {
//        return -1;
//    }
//    
//    _resp->head.len = 1;
//    _resp->errcode = 0;
//    
//    fan_msg_t msg;
//    msg.who = FAN_CTRL_REMOTE;
//    uint8_t speed = (uint8_t)_msg->ubuff.fan.speed_level;
//    if (speed != DF_IGNORE_UINT8) {
//        msg.speed_level = speed;
//        return rt_mq_send(g_fan.mq, &msg, sizeof(fan_msg_t));
//    }
//    return 0;
//}


/**
 * 风扇故障检测
 * @param      : void
 * @return     : void
 */

static void fan_err_detec(void)
{
    static uint8_t s_record_flag = 0;
//    uint8_t txbuf[8] = {0};

    if(g_fan.cur_speed != 0)
    {
        rt_thread_delay(500);  // 风速从转起来到堵转线拉低，可能会有延时，所以这里延时一段时间再检测故障线
        (FAN_RD1_IN_STATUS != 0) ? (g_fan.err_info |= (1<<0)) : (g_fan.err_info &= ~(1<<0));
        (FAN_RD2_IN_STATUS != 0) ? (g_fan.err_info |= (1<<1)) : (g_fan.err_info &= ~(1<<1));
        (FAN_RD3_IN_STATUS != 0) ? (g_fan.err_info |= (1<<2)) : (g_fan.err_info &= ~(1<<2));
        (FAN_RD4_IN_STATUS != 0) ? (g_fan.err_info |= (1<<3)) : (g_fan.err_info &= ~(1<<3));

//                LOG_W("fan rd: 0x%02X", g_fan.err_info);
        if(g_fan.err_info != 0)
        {
//            txbuf[0] = CANCMD_MONITOR_FAN_STATE;
//            txbuf[1] = g_fan.cur_speed;
//            txbuf[2] = g_fan.err_info;
//            can_notify_msg(IDX_CAN1, MONITOR_BOARD_ID, CAN_CONTROL_MODE, txbuf, 3);

            if(s_record_flag == 0) 
            {//只记录一次
                LOG_W("fan error: 0x%02X", g_fan.err_info);
                s_record_flag = 1;
            }
        }
        else
        {
            if(s_record_flag != 0)
            {
                LOG_W("fan error resume");
                s_record_flag = 0;
            }
        }
    }
}
static void fan_task_main(void* _param)
{
    //uint32_t s_timer1 = 0;
    uint32_t s_timer2 = 0;
    
    fan_msg_t fan_msg;
    rt_memset(&fan_msg, 0, sizeof(fan_msg_t));

    uint8_t who_speed_list[FAN_CTRL_WHO_MAX] = {FAN_SPEED_LVL0};
    uint8_t idx_maxsp = FAN_CTRL_NOBODY;
    
    while(1)
    {
        if(fan_mq_recv(&fan_msg, 300) == RT_EOK)
        {
            switch(fan_msg.who)
            {
                case FAN_CTRL_NOBODY:
                    who_speed_list[fan_msg.who] = FAN_SPEED_LVL0;
                break;

                case FAN_CTRL_UNDERPAN:
                case FAN_CTRL_BATERRY :
                    if(g_fan.test_flag != DF_YES) {
                        who_speed_list[fan_msg.who] = fan_msg.speed_level;
                    }
                break;

                case FAN_CTRL_REMOTE:
                    who_speed_list[fan_msg.who] = fan_msg.speed_level;
                break;

                default:
                break;
            }
			
			if(fan_msg.who == FAN_CTRL_REMOTE)
			{
				//远端控制风扇时不再检测其他控制通道(温度)风扇风速
				idx_maxsp = FAN_CTRL_REMOTE;
			}
			else
			{
				//get the max spend level.
				idx_maxsp = FAN_CTRL_NOBODY;
				for(uint8_t m=FAN_CTRL_NOBODY; m<FAN_CTRL_WHO_MAX; m++)
				{
					if(who_speed_list[idx_maxsp] < who_speed_list[m]) {
						idx_maxsp = m;
					}
				}
			}
            
            g_fan.cur_speed = who_speed_list[idx_maxsp];
            fan_set_speed();
        }
/*
        if(os_gettime_ms() - s_timer1 >= 1000/3)
        {
            s_timer1 = os_gettime_ms();
            check_speed();
        } else if (os_gettime_ms() < s_timer1) {
            s_timer1 = os_gettime_ms();
        }
*/        
        if(os_gettime_ms() - s_timer2 >= 1000)
        {
            s_timer2 = os_gettime_ms();
			fan_err_detec();
           /* mcu_data_fan_t fan_data;
            fan_data.speed_level = g_fan.cur_speed;
            //fan_data.rpm = g_fan.rpm;
            fan_data.blocked = 0;
            remote_notify_msg(MCU_DATA_ID2_FAN, (mcu_data_t*)&fan_data, sizeof(mcu_data_fan_t));*/
        } else if (os_gettime_ms() < s_timer2) {
            s_timer2 = os_gettime_ms();
        }
    }
}

int32_t fan_init(void)
{
    rt_memset(&g_fan, 0, sizeof(fan_t));
    
    hw_init();
    
//    remote_service_add(MCU_CMD_ID2_FAN, fan_msg_remote_cb, DF_YES);
    
    g_fan.mq = rt_mq_create("fan_mq", sizeof(fan_msg_t), 8, RT_IPC_FLAG_FIFO);   
    if(g_fan.mq == RT_NULL) {
        return -1;
    }
    
    rt_thread_t fan_thread = rt_thread_create("fan", fan_task_main, RT_NULL, \
                                              TASK_STACK_SIZE_FAN, \
                                              TASK_PRIORITY_FAN, \
                                              20);
    if (fan_thread != RT_NULL)
    {
        rt_thread_startup(fan_thread);
    }
    else
    {
        return -1;
    }
    
    g_fan.is_init = 1;
    return 0;
}


void fan_remote_set(uint8_t speed)
{
	fan_msg_t fan_msg;
	
	fan_msg.who = FAN_CTRL_REMOTE;
	fan_msg.speed_level = speed;
	fan_mq_send(&fan_msg);
}

void fan_status_get(fan_status_t* fan_status)
{
	fan_status->speed_data = g_fan.cur_speed;
	fan_status->err_data = g_fan.err_info;
}

//-----------------------------------------------------------------------------------------------------------------------

#include "finsh.h"

static void fan(uint8_t argc, char **argv)
{
    if(argc != 2)
    {
        rt_kprintf("Please input: fan <0/1/2/3>\n");
    }
    else
    {
        g_fan.test_flag = DF_YES;

        fan_msg_t fan_msg;
        fan_msg.who = FAN_CTRL_REMOTE;
        fan_msg.speed_level = atoi(argv[1]);
        fan_mq_send(&fan_msg);
    }
}
MSH_CMD_EXPORT(fan, fan speed contrl. then fan isnot contrled by temperature.);

static void fan_resume(uint8_t argc, char **argv)
{
    if(argc != 1)
    {
        rt_kprintf("Please input: fan_resume\n");
    }
    else
    {
        g_fan.test_flag = DF_NO;
        
    }
}
MSH_CMD_EXPORT(fan_resume, fan resume contrled by temperature.);









#if 0

/**
 * 远端风速设置
 * @param      : 
 * @return     : void
 */
static void fan_setSpeed_byRemote(uint8_t _node_id, const uint8_t* _prxbuffer, uint8_t* _ptxbuffer, uint8_t* _ptxmsg_len)
{
    if((NULL == _prxbuffer) || (NULL == _ptxbuffer)) {
        return;
    }
    if(_node_id != MONITOR_TEST_ID) {
        return;
    }
  
    g_fan.test_flag = YES;    

    fan_msg_t fan_msg;
    fan_msg.who = FAN_CTRL_CAN;
    fan_msg.speed_level = _prxbuffer[1];
    fan_mq_send(&fan_msg);

    os_delay_ms(20); // 等待设置完成 TODO:

    _ptxbuffer[0] = CANCMD_FAN_CTR_ACK;
    _ptxbuffer[1] = g_fan.cur_speed;

    *_ptxmsg_len = 2;
}

/**
 * 远端查询风速
 * @param      : void
 * @return     : void
 */ 
void fan_queryInfo_byRemote(uint8_t _node_id, const uint8_t* _prxbuffer, uint8_t* _ptxbuffer, uint8_t* _ptxmsg_len)
{
    if((NULL == _prxbuffer) || (NULL == _ptxbuffer))
    {
        return;
    }
    
    if(_node_id != MONITOR_BOARD_ID)
    {
        return;
    }
    
    uint8_t speed = g_fan.cur_speed;
    uint8_t errcode = g_fan.err_info;
    
    _ptxbuffer[0] = CANCMD_MONITOR_FAN_STATE;
    _ptxbuffer[1] = speed;
    
    if(speed != 0) {
        _ptxbuffer[2] = errcode;
    }
    else {
        _ptxbuffer[2] = 0x00;
    }
    
    *_ptxmsg_len = 3;
}


/**
 * 风扇故障检测
 * @param      : void
 * @return     : void
 */

static void fan_err_detec(void)
{
    static uint8_t s_record_flag = 0;
    uint8_t txbuf[8] = {0};

    if(g_fan.cur_speed != 0)
    {
        os_delay_ms(500);  // 风速从转起来到堵转线拉低，可能会有延时，所以这里延时一段时间再检测故障线
        (FAN1_RD_PIN_STATUS != 0) ? (g_fan.err_info |= (1<<0)) : (g_fan.err_info &= ~(1<<0));
        (FAN2_RD_PIN_STATUS != 0) ? (g_fan.err_info |= (1<<1)) : (g_fan.err_info &= ~(1<<1));
        (FAN3_RD_PIN_STATUS != 0) ? (g_fan.err_info |= (1<<2)) : (g_fan.err_info &= ~(1<<2));
        (FAN4_RD_PIN_STATUS != 0) ? (g_fan.err_info |= (1<<3)) : (g_fan.err_info &= ~(1<<3));

        if(g_fan.err_info != 0)
        {
            txbuf[0] = CANCMD_MONITOR_FAN_STATE;
            txbuf[1] = g_fan.cur_speed;
            txbuf[2] = g_fan.err_info;
            can_notify_msg(IDX_CAN1, MONITOR_BOARD_ID, CAN_CONTROL_MODE, txbuf, 3);

            if(s_record_flag == 0) 
            {//只记录一次
                LOG_W("fan error: 0x%02X", g_fan.err_info);
                s_record_flag = 1;
            }
        }
        else
        {
            if(s_record_flag != 0)
            {
                LOG_W("fan error resume");
                s_record_flag = 0;
            }
        }
    }
}

#endif





