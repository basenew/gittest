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
#include "math.h"
#include "common.h"
#include "app_cfg.h"
#include "remote_ctrl.h"
#include "rtdevice.h"
#include "controller.h"
#define LOG_TAG              "remote"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>

#define  YES                               (1)  
#define  NO                                (0) 

#define SIGNAL_UNKNOWN      (0)
#define SIGNAL_SBUS         (1)
#define SIGNAL_PWM          (2)
#define SIGNAL_PPM          (3)  //未严格测试






/* Default config for serial_configure structure */
#define RT_SERIAL_CONFIG_SBUS              \
{                                          \
    100000,           /* 100000 bits/s */  \
    DATA_BITS_8,      /* 8 databits */     \
    STOP_BITS_1,      /* 1 stopbit */      \
    PARITY_NONE,      /* No parity  */     \
    BIT_ORDER_LSB,    /* LSB first sent */ \
    NRZ_NORMAL,       /* Normal mode */    \
    RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
    0                                      \
}


uint32_t g_system_startup_moment;

static  uint8_t remote_recv_mode = SIGNAL_UNKNOWN; 

static uint8_t  g_remote_info_print_flag = NO;
static uint32_t g_remote_info_time_interval = 0;

volatile uint16_t channels[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
static uint8_t  startbyte = 0;
volatile uint8_t  flag      = 0;
static uint8_t  endbyte   = 0;
static uint8_t  g_remote_effect = NO;




//S.BUS-----------------------------------------------------------------------------------------
typedef struct
{
    char*        uart_name;
    rt_device_t  device;
    rt_sem_t     rx_notice;
    rt_err_t     (*uart_rx_ind)(rt_device_t, rt_size_t);
} remote_t;

static rt_err_t rx_ind(rt_device_t dev, rt_size_t size);

static remote_t remote_dev =
{
    .uart_name = "uart2",
    .device    = RT_NULL,
    .rx_notice = RT_NULL,
    .uart_rx_ind = rx_ind,
};

#define UART_RX_EVT  (1<<0)
static rt_err_t rx_ind(rt_device_t _dev, rt_size_t _size)
{
    if(_size > 0) {
        rt_sem_release(remote_dev.rx_notice);
    }

    return RT_EOK;
}

static int32_t device_init(remote_t* _pdev)
{
    struct serial_configure config = RT_SERIAL_CONFIG_SBUS;
	
    //uart 设备
    rt_device_t uart_dev = rt_device_find(_pdev->uart_name);
	rt_device_control(uart_dev, RT_DEVICE_CTRL_CONFIG, &config);
    if(uart_dev != RT_NULL && rt_device_open(uart_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) == RT_EOK)
    {
        _pdev->device = uart_dev;

        rt_device_set_rx_indicate(uart_dev, _pdev->uart_rx_ind);
    }
        
    _pdev->rx_notice = rt_sem_create("sem", 0, RT_IPC_FLAG_FIFO);
    if(_pdev->rx_notice != RT_NULL)
    {
        return 0;
    }    
    LOG_E("fail to open device: %s\n", _pdev->uart_name);
    return -1;
}

static rt_err_t _getchar(remote_t* _pdev, char *ch, int32_t timeout)
{
    rt_err_t result = RT_EOK;

    while (rt_device_read(_pdev->device, 0, ch, 1) == 0)
    {
        rt_sem_control(_pdev->rx_notice, RT_IPC_CMD_RESET, RT_NULL);

        result = rt_sem_take(_pdev->rx_notice, rt_tick_from_millisecond(timeout));
        if (result != RT_EOK)
        {
            return result;
        }
    }

    return RT_EOK;
}

static uint32_t data_read(remote_t* _pdev, uint8_t* _rxbuffer, uint32_t _size, uint32_t _timeout_ms)
{
    rt_size_t read_idx = 0;
    rt_err_t result = RT_EOK;
    char ch = 0;

    RT_ASSERT(_rxbuffer);

    while (1)
    {
        if (read_idx < _size)
        {
            result = _getchar(_pdev, &ch, _timeout_ms);
            if (result != RT_EOK) {
                return 0;
            }
            _rxbuffer[read_idx++] = ch;
        }
        else 
        {
            break;
        }
    }
    return read_idx;
}


void calibrationInter(COMPLEX_def LV,double throttle){
    static COMPLEX_def LV_o = {0.0f,0.0f};
    static rt_bool_t isOpen_ = RT_TRUE;
    static float angle = 0.0f;
    float tem;
    double mod = getComplexMod(LV);
    if((throttle > DF_CALIBRATION_THROTTLE) && isOpen_){
        if((LV_o.real == 0.0f) && (LV_o.imag == 0.0f) && (mod > DF_CALIBRATION_VALUE)){
            LV_o = LV;
            return;
        }
        if(mod < DF_CALIBRATION_VALUE)
            isOpen_ = RT_FALSE;
        tem = getComplexrad(complexDiv(LV,LV_o));
        if(tem > DF_CALIBRATION_ERROR_ANGLE)
            isOpen_ = RT_FALSE;
        angle += tem; 
        rt_kprintf("CALIBRATION:%d\r\n",(int)(angle*100)); 
        LV_o = LV;
        if(angle < DF_CALIBRATION_ENTER_ANGLE){
            rt_kprintf("CALIBRATION:enter\r\n"); 
            if(getCalibrationSwitchStatus())
                enterCalibration();
            isOpen_ = RT_FALSE;
        }
            
    }else{
        
        isOpen_ = RT_FALSE;
    }
    
    
}



void remoteSpeed(int ch1,int ch2,int ch3,int ch4,int flag){
    COMPLEX_def chassisRV;//旋转速度 rad/s
    COMPLEX_def chassisLV;//矢量速度 m/s
    float ch3_float;
    static int flag_old;
    static float throttle = 0.0f;//旋转速度 rad/s
    
    chassisRV.real = 0;
    ch1 = DF_REMOTE_CHANNEL_CENTER - ch1;
    ch2 = DF_REMOTE_CHANNEL_CENTER - ch2;    
    ch3 = DF_REMOTE_CHANNEL_CENTER - ch3;
    ch4 = DF_REMOTE_CHANNEL_CENTER - ch4;

    if(((flag & 0xc) !=0)){
      ch1 = ch2 = ch3 = ch4 = 0; 
      throttle = 0.0f ;  
      chassisLV.real = 0.0f;
      chassisLV.imag = 0.0f;

     chassisRV.imag = 0.0f;
    // if(flag_old == 0)
     //   controlChassis(chassisLV,chassisRV,REMOTE_PRIO);
     flag_old = 1;
      return;
      rt_kprintf("remote normal mode=chassisRV.real = 1==========\r\n");
    } else flag_old = 0;

    ch3_float = (((double)ch3) / DF_REMOTE_CHANNEL_RANGE + 1) * DF_CHASSIS_GEAR_RANGE / 2 + DF_CHASSIS_GEAR_MIN_VALUE;   
    
    if((abs(ch1) < DF_REMOTE_DEAT_ZONE) && 
        (abs(ch2) < DF_REMOTE_DEAT_ZONE) && 
        (abs(ch4) < DF_REMOTE_DEAT_ZONE) ){
        ch1 = ch2 = ch4 = 0;
 
    }
     
    //rt_kprintf("ch1:%d  ch2:%d  ch3:%d  ch4:%d  ch3_float:%d\r\n",ch1,ch2,ch3,ch4,(int)(ch3_float* 100));
     if(((ch3_float < DF_MIN_TURN_ON_THROTTLE) && (throttle < DF_MIN_TURN_ON_THROTTLE)) ||(throttle >=DF_CHASSIS_GEAR_MIN_VALUE))   
        throttle = ch3_float;
     else if(ch3_float > DF_CALIBRATION_THROTTLE){
        chassisLV.real = ((double)ch2)  / DF_REMOTE_CHANNEL_RANGE;
        chassisLV.imag = ((double)ch1)  / DF_REMOTE_CHANNEL_RANGE; 
        calibrationInter(chassisLV,ch3_float);  
        return;
         
     }
     
   

    chassisLV.real = ((double)ch2) * throttle / DF_REMOTE_CHANNEL_RANGE;
    chassisLV.imag = ((double)ch1) * throttle / DF_REMOTE_CHANNEL_RANGE;

    chassisRV.imag = ((double)ch4) * 2 * throttle  / DF_REMOTE_CHANNEL_RANGE;
    //  rt_kprintf("remote real:%d  imag:%d throttle:%d\r\n",ch2,ch1,(int)(ch3_float* 100));      
    //chassisRV.real = 0.0f;
    controlChassis(chassisLV,chassisRV,REMOTE_PRIO);

     
}



static void remote_sbus_info_print(uint8_t* _buf, uint16_t* _channels, uint8_t _flag, uint8_t _startbyte, uint8_t _endbyte)
{
    uint8_t  origin_buf[25] = {0};
    //uint16_t channels[16] = {0};

    rt_memcpy(origin_buf, _buf, sizeof(origin_buf));
    rt_memcpy(channels, _channels, sizeof(channels));

    static uint32_t s_print_timer = 0;
    remoteSpeed(channels[0],channels[1],channels[2],channels[3],_flag);
    if(g_remote_info_print_flag != NO) 
    {
        if(os_gettime_ms() - s_print_timer >= g_remote_info_time_interval)
        {
            s_print_timer = os_gettime_ms();

            rt_kprintf("0x%02X\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t0x%02X\t0x%02X\n", \
            _startbyte, channels[0],channels[1],channels[2],channels[3],channels[4],channels[5],channels[6],channels[7], _flag, _endbyte);
        } else if (os_gettime_ms() < s_print_timer) {
            s_print_timer = os_gettime_ms();
        }
    }
}

static void sbus_decode(uint8_t* _buf, uint16_t* _channels, uint8_t* _flag, uint8_t* _startbyte, uint8_t* _endbyte)
{
	static uint8_t s_flag_cnt = 0;
	
    *_startbyte      =  _buf[0]; 
    *(_channels+0)  = ((_buf[1]|_buf[2]<<8)                     & 0x07FF);        
    *(_channels+1)  = ((_buf[2]>>3 |_buf[3]<<5)                 & 0x07FF);
    *(_channels+2)  = ((_buf[3]>>6 |_buf[4]<<2 |_buf[5]<<10)& 0x07FF);
    *(_channels+3)  = ((_buf[5]>>1 |_buf[6]<<7)                 & 0x07FF);
    *(_channels+4)  = ((_buf[6]>>4 |_buf[7]<<4)                 & 0x07FF);
    *(_channels+5)  = ((_buf[7]>>7 |_buf[8]<<1 |_buf[9]<<9) & 0x07FF);
    *(_channels+6)  = ((_buf[9]>>2 |_buf[10]<<6)                & 0x07FF);
    *(_channels+7)  = ((_buf[10]>>5|_buf[11]<<3)                & 0x07FF);
    *(_channels+8)  = ((_buf[12]   |_buf[13]<<8)                & 0x07FF);
    *(_channels+9)  = ((_buf[13]>>3|_buf[14]<<5)                & 0x07FF);
    *(_channels+10) = ((_buf[14]>>6|_buf[15]<<2|_buf[16]<<10)& 0x07FF);
    *(_channels+11) = ((_buf[16]>>1|_buf[17]<<7)                & 0x07FF);
    *(_channels+12) = ((_buf[17]>>4|_buf[18]<<4)                & 0x07FF);
    *(_channels+13) = ((_buf[18]>>7|_buf[19]<<1|_buf[20]<<9)& 0x07FF);
    *(_channels+14) = ((_buf[20]>>2|_buf[21]<<6)                & 0x07FF);
    *(_channels+15) = ((_buf[21]>>5|_buf[22]<<3)                & 0x07FF);
    *_flag          =   _buf[23];
    *_endbyte       =   _buf[24];
    
    //record when flag changes.
    static uint8_t flag_bk = 0x00;
    if(flag_bk != *_flag)
    {
		if(s_flag_cnt > 4)
		{
			s_flag_cnt = 0;
			LOG_W("sbus flag change,0x%02X--->0x%02X", flag_bk, *_flag);
			
			if(flag_bk == 0x00 && ((*_flag & 0x0C) != 0x00))
			{
			   //if remote control is poweroff or in trouble, brake the car
				
			   //brake_set(BRAKE_CAUSE_REMOTE_OFF, BRAKE_STATUS_LOCK);     
			}

			flag_bk = *_flag;
		}
		else
		{
			s_flag_cnt ++;
		}
    }
    
    remote_sbus_info_print(_buf, _channels, *_flag, *_startbyte, *_endbyte);
}

uint8_t remote_get_sbus_data(uint16_t* _channels)
{
	rt_memcpy(_channels, (const uint16_t*)&channels, sizeof(channels));
	
	return flag;
}
int getChannel(int ch)
{
    if((ch >= (sizeof(channels)/sizeof(uint16_t))) || (ch <0))
        return 0;
    return channels[ch];
}
int getFlag(void)
{
    return flag;
}
#define SBUS_FRAME_LEN   (25)
static void remote_sbus_loop(void)
{
    remote_t* pdev = &remote_dev;
    
    static uint8_t  s_rxbuf[SBUS_FRAME_LEN] = {0};
    static uint8_t  s_readlen = SBUS_FRAME_LEN;
    static uint8_t* s_readptr = s_rxbuf;      
    
    do
    {
        if (data_read(pdev, s_readptr, s_readlen, 100) == s_readlen)
        {
            if (s_rxbuf[0] == 0x0F && s_rxbuf[SBUS_FRAME_LEN-1] == 0x00) 
            {
                s_readlen = SBUS_FRAME_LEN;
                s_readptr = s_rxbuf;
                break;
                
            }
            else
            {
                rt_memcpy(&s_rxbuf[0], &s_rxbuf[1], sizeof(s_rxbuf)-1); // <<
                s_readptr = &s_rxbuf[SBUS_FRAME_LEN-1];
                s_readlen = 1;
            }
        }
        
    } while(1);    

    sbus_decode(s_rxbuf, channels, &flag, &startbyte, &endbyte);
    rt_memset(s_rxbuf, 0, sizeof(s_rxbuf));
	if(g_remote_effect == NO){
		g_remote_effect = YES;
	}
    
}

uint8_t remote_is_effecting(void)
{
	return g_remote_effect;
}


//--------------------------------------------------------------------------------------------------------

static void remote_task_main(void* _param)
{
    while(1)
    {      
        if(remote_recv_mode == SIGNAL_UNKNOWN)
        {
            if(os_gettime_ms() - g_system_startup_moment >= 5000) 
            {

				if(device_init(&remote_dev) == -1) {
					remote_recv_mode = SIGNAL_UNKNOWN;
					LOG_E("init S.BUS uart fail.");
				}
				else
				{
					remote_recv_mode = SIGNAL_SBUS;
					LOG_W("remote:S.BUS");
				}

            }
//            else
//            {
//                rt_thread_mdelay(20);
//            }
        }
        else
        {
            remote_sbus_loop();

        }
        rt_thread_mdelay(20);
    }
}


/**
 * 模块初始化
 * @param      : void
 * @return     : void
 */
int32_t remote_ctrl_init(void)
{   
   g_system_startup_moment = os_gettime_ms();
if(DF_THREAD_STATIC_MEMORY == 0){       
   rt_thread_t thread = rt_thread_create("task_remote", \
                                         remote_task_main, \
                                         RT_NULL, \
                                         TASK_STACK_SIZE_REMOTECTRL, \
                                         TASK_PRIORITY_REMOTECTRL, \
                                         20);
   if (thread != RT_NULL)
   {
       rt_thread_startup(thread);
       rt_kprintf("remote_ctrl_init OK\n");
   }
   else
   {
        rt_kprintf("remote_ctrl_init FALSE\n");
       return -1;
   }
}else{
    static struct rt_thread remote_thread;
    ALIGN(RT_ALIGN_SIZE)
    static char remote_thread_stack[TASK_STACK_SIZE_REMOTECTRL]; 
    	rt_err_t result = RT_EOK;
    result = rt_thread_init(&remote_thread,
                            "task_remote",
                            remote_task_main, RT_NULL,
                            &remote_thread_stack[0], sizeof(remote_thread_stack),
                            TASK_PRIORITY_REMOTECTRL, 20);

    if (result == RT_EOK){
    	rt_thread_startup(&remote_thread);
        return 0;
    }else {
    
        LOG_I("%s thread create failed.",__FUNCTION__);
        return -1;
    } 
}    
    return 0;
}


//---------------------------------------------------------------------------------

#include "finsh.h"

static void remote_info(uint8_t argc, char **argv)
{
    if(argc < 2 || argc > 3)
    {
        rt_kprintf("Please input: remote_info <1/0> <ms, default 0>\n");
    }
    else
    {
        g_remote_info_print_flag = atoi(argv[1]);
        
        if(argc == 3)
        {
            uint32_t _ms = atoi(argv[2]);
//            if(_ms < 100) _ms = 100;
            g_remote_info_time_interval = _ms; 
        }
    }
}
MSH_CMD_EXPORT(remote_info, print sensor information);











