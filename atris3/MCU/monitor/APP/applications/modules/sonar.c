/*
 * COPYRIGHT (C) Copyright 2018-2028; UBT TECH; SHENZHEN, CHINA
 *
 * File       : sonar.c
 * Brief      : sonar超声波数据采集

 * Change Logs
 * Date           Author        Version       Notes
 * 2018-12-18     wuxiaofeng    v1.0
 *
 *
 */

#include "common.h"
#include "board.h"
#include "app_cfg.h"
#include "sonar.h"

#define LOG_TAG              "sonar"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>



#define SONAR_CHANNEL_NUM               4

#define SONAR_CH1         0
#define SONAR_CH2         1
#define SONAR_CH3         2
#define SONAR_CH4         3

#define  SONAR_ERROR_EIGENVALUE_ORIGIN   0xeeee  //传感器发生错误时接收到的特征值
#define  SONAR_ERROR_EIGENVALUE_REPORT   0xFFFF  //上报上层软件的特征值

#define SONAR_INFO_NUMB 11

typedef struct
{
    char*        puart_name;
    rt_device_t  pdevice;
    rt_err_t     (*puart_rx_ind)(rt_device_t, rt_size_t);
    uint32_t     rs485_dir_pin;
    rt_sem_t     rx_notice;
    uint32_t     powerup_time;
    uint32_t     sample_en;
	uint8_t      get_addr;
    uint16_t     version[SONAR_CHANNEL_NUM];
	uint8_t      state[SONAR_CHANNEL_NUM][SONAR_INFO_NUMB];
	uint8_t		 addr[SONAR_CHANNEL_NUM];
	uint8_t      addr_rewrite_cnt[SONAR_CHANNEL_NUM];
	uint8_t      addr_read_cnt[SONAR_CHANNEL_NUM];
} sonar_t;

static uint16_t g_sonardistance_origin[SONAR_CHANNEL_NUM] = {0};  // 原始采集的数据
//static uint16_t g_sonardistance[SONAR_CHANNEL_NUM] = {0};          // 上报远端的数据

static uint8_t  g_sonarcmd_table[4][3] = {
    {0xe8, 0x02, 0x05},    // SONAR_CH1  0
    {0xea, 0x02, 0x05},    // SONAR_CH2  1
    {0xec, 0x02, 0x05},    // SONAR_CH3  2
    {0xee, 0x02, 0x05},    // SONAR_CH4  3
};

static uint8_t g_sonar_addr[4] = {0xe8, 0xea, 0xec, 0xee};

static rt_err_t sonar_ks104_rx_ind(rt_device_t dev, rt_size_t size);

static sonar_t Sonar_KS104 =
{
    .puart_name     = "uart5",
    .pdevice        = RT_NULL,
    .puart_rx_ind   = sonar_ks104_rx_ind,
    .rs485_dir_pin  = SONAR_RS485_DIRPIN_KS104_PI10,
    .rx_notice      = RT_NULL,
    .powerup_time   = 0,
    .sample_en      = 0,
    .version        = 0,
	.state			= {0},
	.addr			= 0,
};


//超声波信息打印相关
static uint8_t  g_sonar_info_print_flag = NO;
static uint32_t g_sonar_info_time_interval = 1000;

static uint8_t g_sonar_check_addr_finish = NO;


static void sonar_power_set_status(uint8_t power_status);


#define UART_RX_EVT  (1<<0)
static rt_err_t sonar_ks104_rx_ind(rt_device_t _dev, rt_size_t _size)
{
    if (_size > 0) {
        rt_sem_release(Sonar_KS104.rx_notice);
    }

    return RT_EOK;
}


rt_inline void sonar_device_rs485_dir_control(sonar_t* _dev, uint8_t _flag)
{
    rt_pin_write(_dev->rs485_dir_pin, _flag);
}


static int32_t sonar_device_init(sonar_t* _dev)
{
    if (_dev == RT_NULL)  return -1;
    if (_dev->puart_name == RT_NULL) return -1;

    //uart 设备
    rt_device_t uart_dev = rt_device_find(_dev->puart_name);
    if (uart_dev != RT_NULL)
    {
        _dev->pdevice = uart_dev;
        rt_device_open(_dev->pdevice, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
        
        if (_dev->puart_rx_ind != RT_NULL)
        {
            rt_device_set_rx_indicate(uart_dev, _dev->puart_rx_ind);
        }

        //初始化rs485 dir pin
        rt_pin_mode(_dev->rs485_dir_pin, PIN_MODE_OUTPUT);
        sonar_device_rs485_dir_control(_dev, PIN_HIGH);

        _dev->rx_notice = rt_sem_create("sonar_sem", 0, RT_IPC_FLAG_FIFO);
        if (_dev->rx_notice != RT_NULL)
        {
            return 0;
        }
    }
    LOG_E("sonar: fail to open device: %s\n", _dev->puart_name);
    return -1;
}

static rt_err_t sonar_getchar(sonar_t* _dev, char *ch, int32_t timeout)
{
    rt_err_t result = RT_EOK;

    while (rt_device_read(_dev->pdevice, 0, ch, 1) == 0)
    {
        rt_sem_control(_dev->rx_notice, RT_IPC_CMD_RESET, RT_NULL);

        result = rt_sem_take(_dev->rx_notice, rt_tick_from_millisecond(timeout));
        if (result != RT_EOK)
        {
            return result;
        }
    }

    return RT_EOK;
}

/**
 * 读串口数据
 * @param      : _rxbuffer 接收缓冲
 * @param      : _size     期望接收数据长度
 * @return     : 0-接收失败   >0-接收数据长度
 */
static uint32_t sonar_data_read(sonar_t* _dev, uint8_t* _rxbuffer, uint32_t _size, uint32_t _timeout_ms)
{
    rt_size_t read_idx = 0;
    rt_err_t result = RT_EOK;
    char ch;

    if(_rxbuffer == RT_NULL) return 0;

    while (1)
    {
        if (read_idx < _size)
        {
            result = sonar_getchar(_dev, &ch, _timeout_ms);
            if (result != RT_EOK)
            {
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

static uint32_t sonar_data_write(sonar_t* _pdev, const uint8_t* _pbuffer, uint32_t _txlen)
{
//        rt_device_control(_pdev->pdevice, errRT_DEVICE_CTRL_CONFIG, RT_NULL);
        sonar_device_rs485_dir_control(_pdev, PIN_HIGH);
        uint32_t tx_size = rt_device_write(_pdev->pdevice, 0, _pbuffer, _txlen);
        sonar_device_rs485_dir_control(_pdev, PIN_LOW);    
        return tx_size;
}

static void ks104_uart_flush(void)
{
	uint8_t temp;
	
	while (rt_device_read(Sonar_KS104.pdevice, 0, &temp, 1) == 1){;}
}

#define  SONAR_RETRY_CNT            3
#define  SONAR_SAMPLE_PERIOD        50
#define  SONAR_SAMPLE_WAITING_TIME  50

static void sonar_data_sample(void)
{
    uint8_t tx_cmd[3] = {0};
    uint8_t rx_buf[4] = {0};
    uint16_t origin_data = SONAR_ERROR_EIGENVALUE_REPORT;
    
    static uint8_t  s_channel_idx = 0;
    static uint32_t s_tx_timer    = 0;
    static uint8_t  s_retry_cnt   = 0;

    if ((Sonar_KS104.sample_en == 1) && os_gettime_ms() - s_tx_timer >= SONAR_SAMPLE_PERIOD)
    {
        s_tx_timer = os_gettime_ms();
				
        rt_memset(rx_buf, 0, sizeof(rx_buf));
        rt_memcpy(tx_cmd, &g_sonarcmd_table[s_channel_idx][0], sizeof(tx_cmd));

		ks104_uart_flush();
		sonar_data_write(&Sonar_KS104, tx_cmd, sizeof(tx_cmd));
		if (sonar_data_read(&Sonar_KS104, rx_buf, sizeof(rx_buf), SONAR_SAMPLE_WAITING_TIME) == sizeof(rx_buf))
		{
			s_retry_cnt = 0;
//			rt_kprintf("sonar_ch-%d, rx_buf-%02X\t%02X\t%02X\t%02X\n",s_channel_idx, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
	
			if((rx_buf[0] == 0xA5) && (rx_buf[3] == (rx_buf[0]^rx_buf[1]^rx_buf[2])))
			{ 
				origin_data = ((((uint16_t)rx_buf[1])<<8) | rx_buf[2]);
                if(origin_data == SONAR_ERROR_EIGENVALUE_ORIGIN){
                    g_sonardistance_origin[s_channel_idx] = SONAR_ERROR_EIGENVALUE_ORIGIN;
                }
                else {
                    g_sonardistance_origin[s_channel_idx] = (uint16_t)(origin_data * 10 / 58);
                }
//				g_sonardistance_origin[s_channel_idx] = origin_data;
			}
		}
		else
		{
			s_retry_cnt ++;
			if (s_retry_cnt > SONAR_RETRY_CNT) {
				s_retry_cnt = 0;
				g_sonardistance_origin[s_channel_idx] = SONAR_ERROR_EIGENVALUE_REPORT;
			}
		}

        if (s_retry_cnt == 0)
        {
            s_channel_idx ++;
        }

        if (s_channel_idx > (SONAR_CHANNEL_NUM - 1))
        {
            s_channel_idx = 0;
        }
    } 
    else if (os_gettime_ms() < s_tx_timer) {
        s_tx_timer = os_gettime_ms();
    }
}

static void sonar_info_print(void)
{
    static uint32_t s_print_timer = 0;

    if (g_sonar_info_print_flag != NO)
    {
        if (os_gettime_ms() - s_print_timer >= g_sonar_info_time_interval)
        {
            s_print_timer = os_gettime_ms();
            rt_kprintf("CH1-%04d CH2-%04d CH3-%04d CH4-%04d\n", \
            g_sonardistance_origin[SONAR_CH1], g_sonardistance_origin[SONAR_CH2], g_sonardistance_origin[SONAR_CH3], g_sonardistance_origin[SONAR_CH4]);
        } 
        else if (os_gettime_ms() < s_print_timer) {
            s_print_timer = os_gettime_ms();
        }
    }
}

static uint8_t g_ks104_readver_flag = 0;
//判断是否可以开始采样
#define SONAR_WHEN_CAN_SAMPLE  2000  //自模块上电后多长时间可以开始采样。因为模块自检需要约1200ms
//初始化完毕模块还会通过串口自动向上位机发送自检数据
static void sonar_sample_enable_check(uint8_t _ch)
{
    uint8_t rx_buf[SONAR_INFO_NUMB] = {0};
    int32_t i = 0;
    

    if(Sonar_KS104.powerup_time == 0)
    {//电源还没打开
		
		Sonar_KS104.powerup_time = os_gettime_ms();
        if(Sonar_KS104.get_addr != 0)
        {
            Sonar_KS104.get_addr = 0;
            sonar_device_rs485_dir_control(&Sonar_KS104, PIN_HIGH);
            //rt_device_close(Sonar_KS104.pdevice);
        }
    }
    else
    {
        if(Sonar_KS104.get_addr == 0 && g_ks104_readver_flag == 0 )
        {//刚打开电源 准备读取版本号
            Sonar_KS104.version[_ch] = 0;
            rt_memset(rx_buf, 0, sizeof(rx_buf));
            sonar_device_rs485_dir_control(&Sonar_KS104, PIN_LOW);   
            sonar_data_read(&Sonar_KS104, rx_buf, SONAR_INFO_NUMB, 1200);
			for(i=0;i<SONAR_INFO_NUMB;i++)
			{
				if(rx_buf[i] != 0)
				{
					Sonar_KS104.version[_ch] = BYTETOSHORT(rx_buf[i], rx_buf[i+1]);
					Sonar_KS104.addr[_ch] = rx_buf[i+3];
					break;
				}
			}
			rt_memcpy(Sonar_KS104.state,rx_buf,SONAR_INFO_NUMB);
            LOG_W("KS104 ch:%d  ver: %04X addr: %04X\n", _ch, Sonar_KS104.version[_ch], Sonar_KS104.addr[_ch] );
            g_ks104_readver_flag = 1;
        }
        else
        {
            if(Sonar_KS104.get_addr == 0)
            {
                if(os_gettime_ms() - Sonar_KS104.powerup_time > SONAR_WHEN_CAN_SAMPLE)
                {
					if(g_ks104_readver_flag)
					{
						Sonar_KS104.get_addr = 1;
						LOG_D("sonar read version over. ch:%d\n", _ch);
					}
                }
            }
        }
    }
}

static void sonar_addr_change(uint8_t addr_current, uint8_t addr_destination)
{
    uint8_t tx_cmd[3] = {0xea, 0x02, 0x9a};

	tx_cmd[0] = addr_current;
	tx_cmd[1] = 0x02;
	tx_cmd[2] = 0x9a;
	sonar_data_write(&Sonar_KS104, tx_cmd, sizeof(tx_cmd));
	rt_thread_delay(10);
	
	tx_cmd[0] = addr_current;
	tx_cmd[1] = 0x02;
	tx_cmd[2] = 0x92;
	sonar_data_write(&Sonar_KS104, tx_cmd, sizeof(tx_cmd));
	rt_thread_delay(10);
	
	tx_cmd[0] = addr_current;
	tx_cmd[1] = 0x02;
	tx_cmd[2] = 0x9e;
	sonar_data_write(&Sonar_KS104, tx_cmd, sizeof(tx_cmd));
	rt_thread_delay(10);
	
	tx_cmd[0] = addr_current;
	tx_cmd[1] = 0x02;
	tx_cmd[2] = addr_destination;
	sonar_data_write(&Sonar_KS104, tx_cmd, sizeof(tx_cmd));
	rt_thread_delay(200);
	LOG_W("SONAR addr write. addr_cur: %02X  addr_dest: %02X\n", addr_current, addr_destination);

}

enum
{
	SONAR_POWER_UP = 0,
	SONAR_READ_ADDR,
	SONAR_CHANGE_ADDR,
	SONAR_REREAD_ADDR,
	SONAR_CHECK_FINISH,
};

static void sonar_check_addr(void)
{
	static uint8_t check_ch = 0;
	static uint8_t sonar_check_addr_step = SONAR_POWER_UP;
	
	switch(sonar_check_addr_step)
	{
		case SONAR_POWER_UP:
			ks104_uart_flush();
			sonar_power_set_status(1<<check_ch);	//打开对应电源
			Sonar_KS104.powerup_time = 0;
            Sonar_KS104.get_addr = 0;
			Sonar_KS104.sample_en = 0;
			g_ks104_readver_flag = 0;
			sonar_check_addr_step = SONAR_READ_ADDR;
			LOG_D("step to read verison.\n");
			break;
		
		case SONAR_READ_ADDR:
			sonar_sample_enable_check(check_ch);
			if(Sonar_KS104.get_addr == 1)
			{
				if(Sonar_KS104.addr[check_ch] == g_sonar_addr[check_ch])	//地址无需改变
				{
					sonar_check_addr_step = SONAR_CHECK_FINISH;
					LOG_D("step to check finish.\n");
				}
				else
				{
					Sonar_KS104.addr_rewrite_cnt[check_ch] ++;
					if(Sonar_KS104.addr_rewrite_cnt[check_ch] > 3)
					{
						sonar_check_addr_step = SONAR_CHECK_FINISH;
						LOG_D("rewrite fail. step to check finish.\n");
					}
					else
					{
						sonar_addr_change(Sonar_KS104.addr[check_ch], g_sonar_addr[check_ch]);
						sonar_check_addr_step = SONAR_REREAD_ADDR;
						LOG_D("step to re-read addr.\n");
					}
				}
			}
			break;
			
		case SONAR_REREAD_ADDR:
			sonar_power_set_status(0);
			rt_thread_delay(200);
			sonar_check_addr_step = SONAR_POWER_UP;
			LOG_D("reread addr:step to power up.\n");
			break;
			
		case SONAR_CHECK_FINISH:
			check_ch ++;
			if(check_ch >= SONAR_CHANNEL_NUM)
			{
				//ID校对完成
				sonar_power_set_status(0x0f);		//打开所有超声电源
				rt_thread_delay(2000);
				g_sonar_check_addr_finish = YES;
				Sonar_KS104.sample_en = 1;
				LOG_W("sonar addr check finish.\n");
			}
			else
			{
				sonar_power_set_status(0);
				rt_thread_delay(200);
				sonar_check_addr_step = SONAR_POWER_UP;
				LOG_D("check next:step to power up.\n");
			}
			break;
			
	}
	
}

static void sonar_task_main(void* _param)
{
    while (1)
    {
		if(g_sonar_check_addr_finish == YES)
		{
			sonar_data_sample();
			sonar_info_print();
		}
		else
		{
			sonar_check_addr();
		}
        rt_thread_delay(10);
    }
}
void sonar_gpio_init(void)
{
    rt_pin_mode(PIN_POWER_12V_KS104_1_PF11,          PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_KS104_2_PH2,           PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_KS104_3_PH3,           PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_POWER_12V_KS104_4_PA0,           PIN_MODE_OUTPUT);
	
	POWER_12V_KS104_1_DISABLE;
	POWER_12V_KS104_2_DISABLE;
	POWER_12V_KS104_3_DISABLE;
	POWER_12V_KS104_4_DISABLE;
	
}

static void sonar_power_set_status(uint8_t power_status)
{
    if(READBIT(power_status,0)) 
	{
		if(!POWER_12V_KS104_1_STATUS) 
		{
			POWER_12V_KS104_1_ENABLE;}} 
		else
		{
			POWER_12V_KS104_1_DISABLE;}
    if(READBIT(power_status,1)) {if(!POWER_12V_KS104_2_STATUS) {POWER_12V_KS104_2_ENABLE;}} else{POWER_12V_KS104_2_DISABLE;}
    if(READBIT(power_status,2)) {if(!POWER_12V_KS104_3_STATUS) {POWER_12V_KS104_3_ENABLE;}} else{POWER_12V_KS104_3_DISABLE;}
    if(READBIT(power_status,3)) {if(!POWER_12V_KS104_4_STATUS) {POWER_12V_KS104_4_ENABLE;}} else{POWER_12V_KS104_4_DISABLE;}
}
int32_t sonar_init(void)
{
    rt_thread_t sonar_thread;

    sonar_device_init(&Sonar_KS104);
	sonar_gpio_init();
    

    sonar_thread = rt_thread_create("task_sonar", \
                                    sonar_task_main, \
                                    RT_NULL, \
                                    TASK_STACK_SIZE_SONAR, \
                                    TASK_PRIORITY_SONAR, \
                                    20);
    if (sonar_thread != RT_NULL)
    {
        rt_thread_startup(sonar_thread);
    }
    else
    {
        return -1;
    }
    return 0;
}



void sonar_get_data(uint16_t* sonar_data)
{
	rt_memcpy(sonar_data, g_sonardistance_origin, sizeof(g_sonardistance_origin));
}


//---------------------------------------------------------------------------------------------------------------------------
#include "finsh.h"
static void sonar_info(uint8_t argc, char **argv)
{
    if (argc < 2 || argc > 3)
    {
        rt_kprintf("Please input: sonar_info <1/0> <ms, default 1000>\n");
    }
    else
    {
        g_sonar_info_print_flag = atoi(argv[1]);

        if (argc == 3)
        {
            uint32_t _ms = atoi(argv[2]);
            if (_ms < 10) _ms = 10;
            g_sonar_info_time_interval = _ms;
        }
    }
}
MSH_CMD_EXPORT(sonar_info, print sonar information);

static void sonar_ver(uint8_t argc, char **argv)
{
    if (argc != 1)
    {
        rt_kprintf("Please input: sonar_ver\n");
    }
    else
    {
        rt_kprintf("KS104 ch1:%04X  ch2:%04X  ch3:%04X  ch4:%04X \n", Sonar_KS104.version[0], Sonar_KS104.version[1], Sonar_KS104.version[2], Sonar_KS104.version[3]);
    }
}
MSH_CMD_EXPORT(sonar_ver, print sonar version);

#if 0
static void config_0x71_0x7d(uint8_t addr,uint8_t data)
{
		uint8_t tx_cmd[][3] = {
			{addr, 0x02, 0x9c}, 
			{addr, 0x02, 0x95}, 
			{addr, 0x02, 0x98}, 
			{addr, 0x02, data/10*16+data%10}, 
		};
		uint16_t rx_buf= 0xffff;
		uint8_t i;
		for(i=0;i<sizeof(tx_cmd)/3;i++)
		{
			rt_thread_delay(100);
			if(addr == 0xea)
			{				
					sonar_data_write(&Sonar_KS104, tx_cmd[i], sizeof(tx_cmd[0]));
					if (sonar_data_read(&Sonar_KS104, (uint8_t *)&rx_buf, sizeof(rx_buf), SONAR_SAMPLE_WAITING_TIME) == sizeof(rx_buf))
					{
						rt_kprintf("%d_TX:	%02X,%02X,%02X RX:%04X pass\n",i,tx_cmd[i][0],tx_cmd[i][1],tx_cmd[i][2],rx_buf);
					}
					else
						rt_kprintf("%d_TX:	%02X,%02X,%02X set para error\n",i,tx_cmd[i][0],tx_cmd[i][1],tx_cmd[i][2]);
			}
			/*else
			{
					sonar_data_write(&Sonar_KS136, tx_cmd[i], sizeof(tx_cmd[0]));
					if (sonar_data_read(&Sonar_KS136, (uint8_t *)&rx_buf, sizeof(rx_buf), SONAR_SAMPLE_WAITING_TIME) == sizeof(rx_buf))
					{
						rt_kprintf("%d_TX:	%02X,%02X,%02X RX:%04X pass\n",i,tx_cmd[i][0],tx_cmd[i][1],tx_cmd[i][2],rx_buf);
					}
					else
						rt_kprintf("%d_TX:	%02X,%02X,%02X set para error\n",i,tx_cmd[i][0],tx_cmd[i][1],tx_cmd[i][2]);
			}*/
		}
		rt_thread_delay(2000);//等待稳定
}

static void sonar_set_para(uint8_t argc, char **argv)
{
		uint8_t i=0;
		uint8_t para=atoi(argv[2]);
		if (para>75 || para<72)
		{
			rt_kprintf("Please input: 106/136 data(72~75)\n");
			return;
		}
    if (argc != 3)
    {
        rt_kprintf("Please input: 106/136 data(72~75)\n");
    }
    else
    {	
				do
				{
					rt_thread_delay(5);
					if(++i > 250)
					{
						rt_kprintf("wait sonar mutex timeout \n");
						return;
					}
				}while(g_sonar_mutex_flag == 1);
				g_sonar_mutex_flag=0;
			
			if(atoi(argv[1]) == 106)
				config_0x71_0x7d(0xea ,para);
			else if(atoi(argv[1]) == 136)
				config_0x71_0x7d(0xe8 ,para);
			else
				rt_kprintf("Please input: 106/136 data(72~75)\n");
			g_sonar_mutex_flag=2;
    }	
}
MSH_CMD_EXPORT(sonar_set_para, set sonar para:106/136 data(72~75));
static void sonar_read_info(uint8_t argc, char **argv)
{
	static const char *sonar_reg_str[]={
		"version   ",
		"date      ",
		"baud_79   ",
		"addr      ",
		"noiselevel",
		"angle     ",
		"errcode_e0",
		"initflag  ",
		"ch12-9    ",
		"ch08-1    ",
		"ch-numb   ",
	};
	uint8_t i;
    if (argc != 1)
    {
        rt_kprintf("Please input: sonar_read_info\n");
    }
    else
    {
				for(i=0;i<SONAR_INFO_NUMB;i++)
				{
//						rt_kprintf("KS106 %s:%02X  KS136 %02X\n",sonar_reg_str[i], Sonar_KS104.state[i],Sonar_KS136.state[i]);
				}
    }
}
MSH_CMD_EXPORT(sonar_read_info, read poweron state);
#endif

