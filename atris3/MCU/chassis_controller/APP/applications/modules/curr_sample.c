#include "rtthread.h"
#include "rtdevice.h"
#include "common.h"
#include "curr_sample.h"

#define PAC_SLAVE_ADRESS (0x20>>1)
#define PAC_I2CBUS_NAME "i2c3"

#define REG_ADDR_PID 0xFD
#define REG_ADDR_MID 0xFE
#define REG_ADDR_RID 0xFF
#define REG_ADDR_PWR 0x1D
#define REG_ADDR_REFRESH 0x00 //0x1F

#define REG_ADDR_SENSE1 0x0B
#define REG_ADDR_SENSE2 0x0C

#define R_MOHM 2

#define CURR_FILTER_NUM	8

typedef struct
{
    uint8_t pid;
    uint8_t mid;
    uint8_t rid;
} pac_data_t;

static pac_data_t g_pac_data;
static uint8_t g_pac_init_ok = 0;
static uint8_t g_print_for_db = 0;
static struct rt_i2c_bus_device *i2c_bus_dev;

//static int16_t curr_buf[CURR_NUM] = {0};
static volatile  uint16_t  g_curr_convert_val[CURR_FILTER_NUM][CURR_NUM];   //电流转换结果
static volatile  uint16_t  g_curr_after_filter_val[CURR_NUM];

static rt_err_t write_reg(uint16_t addr, rt_uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t read_regs(uint16_t addr, rt_uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}


static rt_err_t IDs_read(void)
{
    rt_err_t ret = -RT_ERROR;
    
    ret = read_regs(PAC_SLAVE_ADRESS, REG_ADDR_PID, &g_pac_data.pid, 1);
    ret = read_regs(PAC_SLAVE_ADRESS, REG_ADDR_MID, &g_pac_data.mid, 1);
    ret = read_regs(PAC_SLAVE_ADRESS, REG_ADDR_RID, &g_pac_data.rid, 1);
    rt_kprintf("PAC [PID] %02X [MID] %02X [RID] %02X\n", g_pac_data.pid, g_pac_data.mid, g_pac_data.rid);
    return ret;
}

static rt_err_t chip_cfg(void)
{
    uint8_t data = 0xCC;
    return write_reg(PAC_SLAVE_ADRESS, REG_ADDR_PWR, &data, 1);
}

static rt_err_t refresh(void)
{
    uint8_t data = 0x00;
    return write_reg(PAC_SLAVE_ADRESS, REG_ADDR_REFRESH, &data, 1);
}

static uint16_t get_volt_raw(uint8_t reg, uint8_t avg)
{
    refresh();

    rt_thread_mdelay(2);

    uint8_t offset = 0; //Offset used to select for average

    if(avg) 
      offset = 0x08; //Offset for average registers 

    uint8_t rd[2] = {0}; 
    read_regs(PAC_SLAVE_ADRESS, reg+offset, &rd[0], 2);

    return BYTETOSHORT(rd[0], rd[1]);
}

static uint8_t get_curr_direction(uint8_t ch)
{
    uint8_t rd = 0;
    read_regs(PAC_SLAVE_ADRESS, REG_ADDR_PWR, &rd, 1);
    return (rd >> (7 - ch)) & 0x01; //Return Specified bit
}

//R is in mohms //Have option to take vaerage of last 8 samples 
static int16_t get_current_val(uint8_t ch, double R, uint8_t avg) 
{
    uint8_t dir = get_curr_direction(ch);
    uint16_t vsense = get_volt_raw(REG_ADDR_SENSE1 + ch, avg);

    double FSC = 100.0/R; //Calculate the full scale current, [Amps]
    double curr = 0; 

    if(dir) 
      curr = FSC * ((int16_t)vsense/(32768.0)); //If set for bi-directional, cast to signed value
    else 
      curr = FSC * (vsense/(65536.0));

    return (int16_t)(curr*1000.0); //return mA
}

//adc_average_filter

void curr_read(void)
{
	uint8_t i = 0;
	static uint8_t filter_cnt = 0;
	int32_t  sum = 0;
	
	for(i=0; i<CURR_NUM; i++)
	{
		g_curr_convert_val[filter_cnt][i] = get_current_val(i, R_MOHM, 1);
	}
	filter_cnt ++;
	if(filter_cnt == CURR_FILTER_NUM)
		filter_cnt = 0;
	

    for (uint8_t i = 0; i < CURR_NUM; i++)
    {
        for (uint8_t j = 0; j < CURR_FILTER_NUM; j++)
        {
            sum += g_curr_convert_val[j][i];
        }
        g_curr_after_filter_val[i] = sum / CURR_FILTER_NUM;
        sum = 0;
    }
}

static uint32_t g_curr_info_time_interval = 200;
void curr_print(void)
{
    static uint32_t s_print_timer = 0;
	
    if (g_print_for_db)
	{
        if (os_gettime_ms() - s_print_timer >= g_curr_info_time_interval)
        {
            s_print_timer = os_gettime_ms();
			rt_kprintf("curr-mA:ch1=%d  ch2=%d  ch3=%d  ch4=%d\n", g_curr_after_filter_val[0], g_curr_after_filter_val[1], g_curr_after_filter_val[2], g_curr_after_filter_val[3]); 
        } else if (os_gettime_ms() < s_print_timer) {
            s_print_timer = os_gettime_ms();
        }
    }
}

static rt_err_t device_init(void)
{
    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(PAC_I2CBUS_NAME);
    if (i2c_bus_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    //i2c_bus_dev->addr = (PAC_SLAVE_ADRESS) & 0xFF;
    
    return RT_EOK;
}

int32_t curr_sample_init(void)
{
    device_init();
    
    if (IDs_read() == RT_EOK) 
    {
        if (chip_cfg() == RT_EOK) {
            g_pac_init_ok = 1;
        }
    }
    return 0;
}


uint16_t curr_get_data(uint8_t ch)
{
	return g_curr_after_filter_val[ch];
}
//------------------------------------------------------------------------------------------
static void curr(uint8_t argc, char **argv)
{
    #define PRINT_INFO \
    { \
        rt_kprintf("Please input: curr <1/0>\n"); \
    }
    if (argc != 2)
    {
        PRINT_INFO;
    }
    else
    {
        g_print_for_db = atoi(argv[1]);

    }
}
MSH_CMD_EXPORT(curr, print current sensor);



















