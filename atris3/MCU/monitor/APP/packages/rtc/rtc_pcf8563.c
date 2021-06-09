/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : rtc_pcf8563.c
 * Brief      : RTC PCF8563 driver

 * Change Logs
 * Date           Author          Notes
 * 2019-06-04     wuxiaofeng      first version        
 * 2019-10-21     wuxiaofeng      port to RT-thread RTC device
 */

#include "board.h"

#include "rtc_pcf8563.h"

#define RTC_ALARM_REG_DISABLE_MASK  (1<<7)
#define RTC_ALARM_REG_ENABLE_MASK   (~RTC_ALARM_REG_DISABLE_MASK)


//#define DRV_DEBUG
#define LOG_TAG             "rtc"
#include <drv_log.h>

#define REG_CONTROL_STATUS_1       0x00
#define REG_CONTROL_STATUS_2       0x01

#define REG_VL_SECOND              0x02
#define REG_MINUTE                 0x03
#define REG_HOUR                   0x04
#define REG_DAY                    0x05
#define REG_WEEKDAY                0x06
#define REG_CENTURY_MONTH          0x07
#define REG_YEAR                   0x08

#define REG_MINUTE_ALARM           0x09
#define REG_HOUR_ALARM             0x0A
#define REG_DAY_ALARM              0x0B
#define REG_WEEKDAY_ALARM          0x0C

#define REG_CLKOUT_CTR             0x0D
#define REG_TIMER_CTR              0x0E
#define REG_TIMER                  0x0F

#define BIT_AIE                    (1<<1)
#define BIT_AF                     (1<<3)

#define BCDTODECIMAL(BYTEDATA)              ((((BYTEDATA)>>4)*10)+((BYTEDATA)&0x0F))
#define DECIMALTOBCD(BYTEDATA)              ((((BYTEDATA/10))<<4)+((BYTEDATA)%10))

typedef struct
{
    char* inf_name;
    rt_uint8_t w_addr;
    rt_uint8_t r_addr;
} rtc_inf_t;

static rtc_inf_t rtc_inf = 
{
    RTC_USING_I2CBUS_NAME,
    (0xA2>>1),
    (0xA3>>1),
};
static struct rt_i2c_bus_device *i2c_bus_dev;

static struct rt_device rtc;

static rt_err_t write_regs(rt_uint8_t reg, rt_uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    rtc_inf_t* pinf = &rtc_inf;
    
    msgs[0].addr  = pinf->w_addr;     /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = pinf->w_addr;     /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t read_regs(rt_uint8_t reg, rt_uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    rtc_inf_t* pinf = &rtc_inf;

    msgs[0].addr  = pinf->w_addr;     /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = pinf->r_addr;     /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t rt_rtc_init(rtc_inf_t* _inf)
{
    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(_inf->inf_name);
    if (i2c_bus_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    //i2c_bus_dev->addr = (rt_uint16_t)(_inf->addr & 0xFF);

    return RT_EOK;
}


static rt_err_t set_rtc_time_stamp(struct tm *p_tm)
{
    rt_uint8_t buf[7] = {0};
    buf[0] = DECIMALTOBCD(p_tm->tm_sec) & 0x7F; 
    buf[1] = DECIMALTOBCD(p_tm->tm_min) & 0x7F;
    buf[2] = DECIMALTOBCD(p_tm->tm_hour)& 0x3F;
    buf[3] = DECIMALTOBCD(p_tm->tm_mday)& 0x3F;
    buf[4] = DECIMALTOBCD(p_tm->tm_wday)& 0x07;
    buf[5] = DECIMALTOBCD(p_tm->tm_mon) & 0x1F;
    buf[6] = DECIMALTOBCD(p_tm->tm_year)& 0xFF;

    LOG_D("set rtc time.");
    return write_regs(REG_VL_SECOND, buf, 7);
}


static time_t get_rtc_time_stamp(void)
{
    struct  tm tm_new;
    rt_uint8_t buf[7] = {0};

    if(read_regs(REG_VL_SECOND, buf, 7) != RT_EOK) {
        return (time_t)-1;
    }

    tm_new.tm_sec   = BCDTODECIMAL(buf[0]&0x7F);
    tm_new.tm_min   = BCDTODECIMAL(buf[1]&0x7F);
    tm_new.tm_hour  = BCDTODECIMAL(buf[2]&0x3F);
    tm_new.tm_mday  = BCDTODECIMAL(buf[3]&0x3F);
    tm_new.tm_wday  = BCDTODECIMAL(buf[4]&0x07);
    tm_new.tm_mon   = BCDTODECIMAL(buf[5]&0x1F);
    tm_new.tm_year  = BCDTODECIMAL(buf[6]&0xFF);
    LOG_D("get rtc time.");
    return mktime(&tm_new);
}

static rt_err_t set_rtc_alarm_stamp(struct tm* p_tm)
{
    rt_uint8_t buf[4] = {0};
    
    buf[0] = DECIMALTOBCD(p_tm->tm_min) & 0x7F;
    if(p_tm->tm_min & RTC_ALARM_REG_DISABLE_MASK) {
        buf[0] |= RTC_ALARM_REG_DISABLE_MASK;
    }
    
    buf[1] = DECIMALTOBCD(p_tm->tm_hour) & 0x3F;
    if(p_tm->tm_hour & RTC_ALARM_REG_DISABLE_MASK) {
        buf[1] |= RTC_ALARM_REG_DISABLE_MASK;
    }
    
    buf[2] = DECIMALTOBCD(p_tm->tm_mday) & 0x3F;
    if(p_tm->tm_mday & RTC_ALARM_REG_DISABLE_MASK) {
        buf[2] |= RTC_ALARM_REG_DISABLE_MASK;
    }
    
    buf[3] = DECIMALTOBCD(p_tm->tm_wday) & 0x07;
    if(p_tm->tm_wday & RTC_ALARM_REG_DISABLE_MASK) {
        buf[3] |= RTC_ALARM_REG_DISABLE_MASK;
    }
    
    LOG_D("set rtc alarmtime.");
    return write_regs(REG_MINUTE_ALARM, buf, 4);
}

static rt_err_t get_rtc_alarm_stamp(struct tm* p_tm)
{
    rt_uint8_t buf[4] = {0};

    if(read_regs(REG_MINUTE_ALARM, buf, 4) != RT_EOK) {
        return -RT_ERROR;
    }
    
    p_tm->tm_min   = BCDTODECIMAL(buf[0]&0x7F);
    if(buf[0] & RTC_ALARM_REG_DISABLE_MASK) {
        p_tm->tm_min |= RTC_ALARM_REG_DISABLE_MASK;
    }
    
    p_tm->tm_hour  = BCDTODECIMAL(buf[1]&0x3F);
    if(buf[1] & RTC_ALARM_REG_DISABLE_MASK) {
        p_tm->tm_hour |= RTC_ALARM_REG_DISABLE_MASK;
    }
    
    p_tm->tm_mday  = BCDTODECIMAL(buf[2]&0x3F);
    if(buf[2] & RTC_ALARM_REG_DISABLE_MASK) {
        p_tm->tm_mday |= RTC_ALARM_REG_DISABLE_MASK;
    }
    
    p_tm->tm_wday  = BCDTODECIMAL(buf[3]&0x07);
    if(buf[3] & RTC_ALARM_REG_DISABLE_MASK) {
        p_tm->tm_wday |= RTC_ALARM_REG_DISABLE_MASK;
    }
    LOG_D("get rtc alarmtime.");
    return RT_EOK;
}

static rt_err_t set_alarm_AF(rt_uint8_t flag)
{
    rt_uint8_t buf[1] = {0};

    if(read_regs(REG_CONTROL_STATUS_2, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }
    if(flag == 0) {
        buf[0] &= ~BIT_AF;
    }
    else {
        buf[0] |= BIT_AF;
    }
    return write_regs(REG_CONTROL_STATUS_2, buf, 1);
}

static rt_err_t set_alarm_AIE(rt_uint8_t flag)
{
    rt_uint8_t buf[1] = {0};
    if(read_regs(REG_CONTROL_STATUS_2, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }
    if(flag == 0) {
        buf[0] &= ~BIT_AIE;
    }
    else {
        buf[0] |= BIT_AIE;
    }
    return write_regs(REG_CONTROL_STATUS_2, buf, 1);
}

static rt_err_t rt_rtc_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    RT_ASSERT(dev != RT_NULL);
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        *(rt_uint32_t *)args = get_rtc_time_stamp();
        if(*(rt_uint32_t *)args == (time_t)-1)
        {
            result = -RT_ERROR;
        }
        LOG_D("RTC: get rtc time %x\n", *(rt_uint32_t *)args);
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
        result = set_rtc_time_stamp((struct tm*)args);
        LOG_D("RTC: set rtc time %d\n", result);
        break;
    
    case RT_DEVICE_CTRL_RTC_SET_ALARM:
        result = set_rtc_alarm_stamp((struct tm *)args);
        LOG_D("RTC: set rtc alarm %d\n", result);
        break;

    case RT_DEVICE_CTRL_RTC_GET_ALARM:
        result = get_rtc_alarm_stamp((struct tm *)args);
        LOG_D("RTC: get rtc alarm %d\n", result);
        break;

    case RT_DEVICE_CTRL_RTC_SET_ALARM_AF:
        result = set_alarm_AF(*(rt_uint8_t *)args);
        LOG_D("RTC: set alarm AF %x\n", *(rt_uint8_t *)args);
        break;

    case RT_DEVICE_CTRL_RTC_SET_ALARM_AIE:
        result = set_alarm_AIE(*(rt_uint8_t *)args);
        LOG_D("RTC: set alarm AIE %x\n", *(rt_uint8_t *)args);
        break;

    default: break;
    }

    return result;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rtc_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    rt_rtc_control
};
#endif


static rt_err_t rt_hw_rtc_register(rt_device_t device, const char *name, rt_uint32_t flag)
{
    RT_ASSERT(device != RT_NULL);

    if (rt_rtc_init(&rtc_inf) != RT_EOK)
    {
        return -RT_ERROR;
    }
#ifdef RT_USING_DEVICE_OPS
    device->ops         = &rtc_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_rtc_control;
#endif
    device->type        = RT_Device_Class_RTC;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->user_data   = RT_NULL;

    /* register a character device */
    return rt_device_register(device, name, flag);
}

int rt_hw_rtc_init_pcf8563(void)
{
    rt_err_t result;
    result = rt_hw_rtc_register(&rtc, "rtc", RT_DEVICE_FLAG_RDWR);
    if (result != RT_EOK)
    {
        LOG_E("rtc register err code: %d", result);
        return result;
    }
    LOG_D("rtc init success");
    return RT_EOK;
}
//INIT_DEVICE_EXPORT(rt_hw_rtc_init);









