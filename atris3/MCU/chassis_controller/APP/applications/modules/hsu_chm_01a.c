/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : hsu_chm_01.c
 * Brief      : 温湿度传感器HSU_CHM_01A驱动

 * Change Logs
 * Date           Author        Version       Notes
 * 2019-03-01     wuxiaofeng    v1.0          
 *
 */

#include "sensor.h"
#include "hsu_chm_01a.h"


#ifdef PKG_USING_HSUCHM_01A

#define LOG_TAG              "hsu"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>

#define SLAVE_ADRESS 0x7F
#define SLAVE_ADRESS_BYTE0 0x00 //RESET
#define SLAVE_ADRESS_BYTE1 0x01 //MAN:TAVE:HAVE:MANMODE:
#define SLAVE_ADRESS_BYTE2 0x02 //RESERVED
#define SLAVE_ADRESS_BYTE3 0x03 //ERR:
#define SLAVE_ADRESS_BYTE4 0x04 //Humidity detection 
#define SLAVE_ADRESS_BYTE5 0x05 //Humidity detection 
#define SLAVE_ADRESS_BYTE6 0x06 //temperature detection 
#define SLAVE_ADRESS_BYTE7 0x07 //temperature detection 


static struct rt_i2c_bus_device *i2c_bus_dev;


static int write_reg(uint16_t addr, rt_uint8_t reg, uint8_t *data, uint16_t len)
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

static int read_regs(uint16_t addr, rt_uint8_t reg, uint8_t *data, uint16_t len)
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


static rt_err_t hsu_detection_start(void)
{
    uint16_t start_retry = 3;
    
    uint8_t buf[1] = {0x01};
    write_reg(i2c_bus_dev->addr, SLAVE_ADRESS_BYTE1, buf, 1); // Detection operation start
    /*try 3 times to get connect the IIC. 
		issue: IIC broken when current and voltage slave sensor*/
    while( ((buf[0]&0x01) != 0) && start_retry > 0)
    {
        rt_thread_mdelay(5);
        read_regs(i2c_bus_dev->addr, SLAVE_ADRESS_BYTE1, buf, 1);
        start_retry --;
    }
    if(start_retry == 0)
    {
        LOG_W("hsu sensor start_retry fail.");
        return -RT_ERROR;
    }
    
    buf[0] = 0x01;
    read_regs(i2c_bus_dev->addr, SLAVE_ADRESS_BYTE3, buf, 1);
    if((buf[0]&0x01) == 0x01)
    {   
        buf[0] = 0x01;
        write_reg(i2c_bus_dev->addr, SLAVE_ADRESS_BYTE3, buf, 1);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t hsu_polling_get_data(struct rt_sensor_device *sensor, void *buf)
{
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    
    if(hsu_detection_start() != RT_EOK) 
    {
        return -RT_ERROR;
    }
    
    if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {
        uint8_t temp_val[2] = {0};

        read_regs(i2c_bus_dev->addr, SLAVE_ADRESS_BYTE6, &temp_val[0], 1);  
        read_regs(i2c_bus_dev->addr, SLAVE_ADRESS_BYTE7, &temp_val[1], 1); 

        data->type = RT_SENSOR_CLASS_TEMP;
        data->data.temp = ((uint16_t)temp_val[0] + ((uint16_t)temp_val[1]<<8));
        
        data->data.temp -= 774;  // T = ( Tc - (1024 - 250) ) * 0.1
        
        if(data->data.temp < sensor->info.range_min*10) data->data.temp = sensor->info.range_min*10;
        if(data->data.temp > sensor->info.range_max*10) data->data.temp = sensor->info.range_max*10;
        
        data->timestamp = 0;//rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_HUMI)
    {
        uint8_t humi_val[2] = {0};

        read_regs(i2c_bus_dev->addr, SLAVE_ADRESS_BYTE4, &humi_val[0], 1); 
        read_regs(i2c_bus_dev->addr, SLAVE_ADRESS_BYTE5, &humi_val[1], 1); 
        
        data->type = RT_SENSOR_CLASS_HUMI;
        data->data.humi = ((uint16_t)humi_val[0] + ((uint16_t)humi_val[1]<<8));
        
        data->data.humi = (data->data.humi * 100 / 1024) * 10;
        
        if(data->data.temp < sensor->info.range_min*10) data->data.temp = sensor->info.range_min*10;
        if(data->data.temp > sensor->info.range_max*10) data->data.temp = sensor->info.range_max*10;
        
        data->timestamp = 0;//rt_sensor_get_ts();
    }
    else
        return -RT_ERROR;
    
    return RT_EOK;
}

static rt_err_t hsu_device_init(struct rt_sensor_intf *intf)
{
    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(intf->dev_name);
    if (i2c_bus_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    i2c_bus_dev->addr = (rt_uint32_t)(intf->user_data) & 0xFF;
    
    hsu_detection_start();
    
    return RT_EOK;
}

static rt_size_t hsu_temp_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct rt_sensor_data *databuf = (struct rt_sensor_data *)buf;
    
    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        rt_size_t recvsize = len;
        while(recvsize)
        {
            if(hsu_polling_get_data(sensor, databuf) == RT_EOK)
            {
                databuf ++;
                recvsize --;
            }
            else
            {
                break;
            }
        }
        return (len - recvsize);
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_INT)
    {
        return 0;
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_FIFO)
    {
        return 0;
    }
    else
        return 0;
}


static rt_err_t hsu_temp_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        
        break;
    
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops hsu_temp_ops =
{
    hsu_temp_fetch_data,
    hsu_temp_control
};

int rt_hw_hsu_temp_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -1;

    sensor->info.type       = RT_SENSOR_CLASS_TEMP;
    sensor->info.vendor     = RT_SENSOR_VENDOR_HDK;
    sensor->info.model      = "hsu_temp";
    sensor->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = 100;
    sensor->info.range_min  = -30;
    sensor->info.period_min = 0;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &hsu_temp_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("hsu_humi register err code: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }
    return 0;
}

int rt_hw_hsu_humi_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -1;

    sensor->info.type       = RT_SENSOR_CLASS_HUMI;
    sensor->info.vendor     = RT_SENSOR_VENDOR_HDK;
    sensor->info.model      = "hsu_humi";
    sensor->info.unit       = RT_SENSOR_UNIT_PERMILLAGE;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = 100;
    sensor->info.range_min  = 0;
    sensor->info.period_min = 0;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &hsu_temp_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("hsu_humi register err code: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }
    return 0;
}

int hsu_init(void)
{
    struct rt_sensor_config cfg;
    
    rt_memset(&cfg, 0, sizeof(struct rt_sensor_config));

    cfg.intf.dev_name  = "i2c3";
    cfg.intf.type      = RT_SENSOR_INTF_I2C;
    cfg.intf.user_data = (void *)SLAVE_ADRESS; 
    
    if(hsu_device_init(&(cfg.intf)) != RT_EOK)
    {
        LOG_E("hsu sensor inf init err.");
        return -1;
    }
    cfg.mode = RT_SENSOR_MODE_POLLING;
    
#ifdef PKG_USING_HSUCHM_01A_TEMP 
    rt_hw_hsu_temp_init("hsu", &cfg);
#endif  
    
#ifdef PKG_USING_HSUCHM_01A_HUMI 
    rt_hw_hsu_humi_init("hsu", &cfg);
#endif  
    
    return RT_EOK;
}
INIT_APP_EXPORT(hsu_init);


#endif  // #ifdef PKG_USING_HSUCHM_01A

















