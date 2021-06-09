/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : rt_fota.c
 * Brief      : 

 * Change Logs
 * Date           Author          Notes
 * 2020-05-05     wuxiaofeng      first version        
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <fal.h>
#include <rt_fota.h>
#include "led.h"

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>
#include <shell.h>
#endif

#define DBG_ENABLE
#define DBG_SECTION_NAME                    "fota"

#ifdef RT_FOTA_DEBUG
#define DBG_LEVEL                           DBG_LOG
#else
#define DBG_LEVEL                           DBG_INFO
#endif

#define DBG_COLOR
#include <rtdbg.h>

#ifndef RT_FOTA_THREAD_STACK_SIZE			
#define RT_FOTA_THREAD_STACK_SIZE			4096
#endif

#ifndef RT_FOTA_THREAD_PRIORITY			
#define RT_FOTA_THREAD_PRIORITY				(RT_THREAD_PRIORITY_MAX - 3)
#endif

#ifndef RT_FOTA_ALGO_BUFF_SIZE
#define RT_FOTA_ALGO_BUFF_SIZE				4096
#endif


#ifndef RT_FOTA_ENTER_SHELL_KEY
#define RT_FOTA_ENTER_SHELL_KEY				0x0d
#endif

#ifndef RT_FOTA_GET_CHAR_WAITTIGN
#define RT_FOTA_GET_CHAR_WAITTIGN			(RT_TICK_PER_SECOND * 2)
#endif


typedef struct {
	char type[4];              /*字符头 RT_FOTA_HEADER_TYPE*/
	rt_uint16_t fota_algo;     /*算法配置 加密/压缩*/
	rt_uint8_t fm_time[6];	   /*固件时间戳 */
	char target_part_name[16]; /*固件目标分区 app/bl*/
	char software_version[24]; /*固件版本号*/
	char hardware_version[24]; /*适配硬件版本号*/
	rt_uint32_t code_crc;      /*固件body CRC*/
	rt_uint32_t reserved;      /*保留*/
	rt_uint32_t raw_size;      /*原始body的大小*/
	rt_uint32_t com_size;      /*打包后body的大小*/
	rt_uint32_t head_crc;      /*header的CRC*/
} rt_fota_part_head, *rt_fota_part_head_t;


ALIGN(1)
static rt_fota_part_head fota_part_head;
static const struct fal_partition *target_part = RT_NULL;


static int rt_fota_boot_init(void)
{
	int fota_res = RT_FOTA_NO_ERR;

	rt_memset(&fota_part_head, 0x0, sizeof(rt_fota_part_head));
	
	/* partition initial */
	fal_init(); 

	extern int fal_init_check(void);
	/* verify partition */
	if (fal_init_check() != 1)
    {
    	LOG_D("Partition initialized failed!");
		fota_res = RT_FOTA_FAL_CHECK_ERR;
		goto __exit_boot_verify;
    }

__exit_boot_verify:
	return fota_res;
}

enum E_CHECK_FLAG
{
    H_CHECK_ALL = 0,
    H_NO_CHECK_TARGET_NAME = (1u<<0),
};

static int rt_fota_part_fw_verify(const char *part_name, uint8_t chk_flag)
{
#define RT_FOTA_CRC_BUFF_SIZE		4096
#define RT_FOTA_CRC_INIT_VAL		0xffffffff
    
	int fota_res = RT_FOTA_NO_ERR;
	const struct fal_partition *part;
	rt_fota_part_head part_head;
	rt_uint8_t *body_buf = RT_NULL;
	rt_uint32_t body_crc = RT_FOTA_CRC_INIT_VAL;
	rt_uint32_t hdr_crc;
    
	if (part_name == RT_NULL)
	{
		LOG_I("Invaild paramenter input!");
		fota_res = RT_FOTA_HEADER_CHECK_ERR;
		goto __exit_partition_verify;
	}

	part = fal_partition_find(part_name);
	if (part == RT_NULL)
	{		
		LOG_I("Partition[%s] not found.", part_name);
		fota_res = RT_FOTA_HEADER_FIND_ERR;
		goto __exit_partition_verify;
	}

	/* read the header */
	if (fal_partition_read(part, 0, (rt_uint8_t *)&part_head, sizeof(rt_fota_part_head)) < 0)
	{
		LOG_I("Partition[%s] read error!", part->name);
		fota_res = RT_FOTA_HEADER_READ_ERR;
		goto __exit_partition_verify;
	}

	/*check head crc*/
	extern rt_uint32_t rt_fota_crc(rt_uint8_t *buf, rt_uint32_t len);
	hdr_crc = rt_fota_crc((rt_uint8_t *)&part_head, sizeof(rt_fota_part_head) - 4);
	if (hdr_crc != part_head.head_crc)
	{
		LOG_I("Partition[%s] head CRC32 error!", part->name);
		fota_res = RT_FOTA_HEADER_H_CRC_ERR;
		goto __exit_partition_verify;
	}
	
	/*check head type*/
	if (rt_strcmp(part_head.type, RT_FOTA_HEADER_TYPE) != 0)
	{
		LOG_I("Partition[%s] type [%s] not surport.", part->name, part_head.type);
		fota_res = RT_FOTA_HEADER_TYPE_ERR;
		goto __exit_partition_verify;
	}

	/*in app, allow to update bootloader only*/
	if (!(chk_flag & H_NO_CHECK_TARGET_NAME) && rt_strcmp(part_head.target_part_name, RT_FOTA_BL_PART_NAME)) 
	{
		LOG_I("Partition[%s] updating is not surport in app.", part_head.target_part_name);
		fota_res = RT_FOTA_HEADER_SUPPORT_ERR;
		goto __exit_partition_verify;
	}

	/*check partition is exist or not*/
    target_part = fal_partition_find(part_head.target_part_name);
	if (target_part == RT_NULL)
	{
		LOG_I("Partition[%s] not found.", part_head.target_part_name);
		fota_res = RT_FOTA_HEADER_CHECK_ERR;
		goto __exit_partition_verify;
	}

	/*check body crc*/
	body_buf = rt_malloc(RT_FOTA_CRC_BUFF_SIZE);
	if (body_buf == RT_NULL)
	{
		LOG_I("Not enough memory for body CRC32 verify.");	
		fota_res = RT_FOTA_HEADER_NO_MEN_ERR;
		goto __exit_partition_verify;
	}

	for (int body_pos = 0; body_pos < part_head.com_size;)
	{	
		int body_read_len = fal_partition_read(part, sizeof(rt_fota_part_head) + body_pos, body_buf, RT_FOTA_CRC_BUFF_SIZE);      
		if (body_read_len > 0) 
		{
            if ((body_pos + body_read_len) > part_head.com_size)
            {
                body_read_len = part_head.com_size - body_pos;
            }
            
			extern rt_uint32_t rt_fota_step_crc(rt_uint32_t crc, rt_uint8_t *buf, rt_uint32_t len);
			body_crc = rt_fota_step_crc(body_crc, body_buf, body_read_len);	
			body_pos = body_pos + body_read_len;
		}
		else
		{
			LOG_I("Partition[%s] read error!", part->name);		
			fota_res = RT_FOTA_HEADER_READ_ERR;
			goto __exit_partition_verify;
		}
	}
	body_crc = body_crc ^ RT_FOTA_CRC_INIT_VAL;
	
	if (body_crc != part_head.code_crc)
	{
		LOG_I("Partition[%s] firmware integrity verify failed.", part->name);		
		fota_res = RT_FOTA_HEADER_B_CRC_ERR;
		goto __exit_partition_verify;
	}

__exit_partition_verify:
	if (fota_res == RT_FOTA_NO_ERR)
	{
		rt_memcpy(&fota_part_head, &part_head, sizeof(rt_fota_part_head));
		LOG_D("partition[%s] verify success!", part->name);
	}
	else
	{
		rt_memset(&fota_part_head, 0x0, sizeof(rt_fota_part_head));
		LOG_I("Partition[%s] verify failed!", part->name);
	}

	if (body_buf)
		rt_free(body_buf);
	
	return fota_res;
}


int rt_fota_check_upgrade(void)
{
	int fota_res = RT_FOTA_NO_ERR;
	fota_flags_t* pflags = fota_flags_get();

	if (pflags->uflag.sflag.force_up == FLAG_YES) {
		LOG_D("app needs upgrade, by forced.");
		fota_res = RT_FOTA_NO_ERR;
		goto __exit_check_upgrade;
	}
    
    {
        /*check hardware version*/
        int real_hw_ver = (int)hardware_version_get();

        int  header_hw_ver[24] = {0};
        char c_temp[sizeof(fota_part_head.hardware_version)] = {0};
        rt_memcpy(c_temp, fota_part_head.hardware_version, sizeof(c_temp));

        uint8_t cnt = 0;
        uint8_t is_fond_hw_ver = 0;
        char *token = RT_NULL;
        const char s[2]= ",";
        token = strtok(c_temp, s);

        while (token != RT_NULL)
        {
            header_hw_ver[cnt++] = atoi(token);
            token = strtok(RT_NULL, s);
        }
        
        if (cnt == 0) {
            //if not Specified hardware versions, all pass
            is_fond_hw_ver = 1;
        }
        else {
            for (uint8_t i = 0; i < cnt; i++) {
                if (real_hw_ver == header_hw_ver[i]) {
                    is_fond_hw_ver = 1;
                    break;
                }
            }
        }

        if (is_fond_hw_ver == 0) {
            fota_res = RT_FOTA_HEADER_HW_CHECK_ERR;
            LOG_I("hardware versions not support. hw_ver: %d, fw_ver: %s", real_hw_ver, fota_part_head.hardware_version);
            goto __exit_check_upgrade;
        }
    }

    if (!rt_strcmp(fota_part_head.software_version, pflags->bl_version))
    {
        fota_res = RT_FOTA_HEADER_SW_CHECK_ERR;
        LOG_I("software version does not need upgrade.");
        goto __exit_check_upgrade;
    }

__exit_check_upgrade:
	return fota_res;
}

int rt_fota_erase_app_part(void)
{
	int fota_res = RT_FOTA_NO_ERR;
	const struct fal_partition *part = RT_NULL;

	part = target_part;
	if (part == RT_NULL)
	{
		LOG_I("Erase partition[%s] not found.", fota_part_head.target_part_name);
		fota_res = RT_FOTA_UPGRADE_CHECK_ERR;
		goto __exit_partition_erase;
	}
    
    LOG_I("Partition[%s] erase start:", part->name);
	if (fal_partition_erase(part, 0, fota_part_head.raw_size) < 0)
	{
		LOG_I("Partition[%s] erase failed!", part->name);
		fota_res = RT_FOTA_UPGRADE_ERASE_ERR;
		goto __exit_partition_erase;
	}

__exit_partition_erase:
	if (fota_res == RT_FOTA_NO_ERR)
	{
		LOG_D("Partition[%s] erase %d bytes success!", part->name, fota_part_head.raw_size);
	}
	return fota_res;
}

int rt_fota_write_app_part(int fw_pos, rt_uint8_t *fw_buf, int fw_len)
{
	int wr_len = -1;
	const struct fal_partition *part;

	part = target_part;
	if ((part == RT_NULL) || (fw_buf == RT_NULL) || (fw_len > RT_FOTA_ALGO_BUFF_SIZE))
	{
		goto __partition_write_exit;
	}

	wr_len = fal_partition_write(part, fw_pos, fw_buf, fw_len);

__partition_write_exit:
	return wr_len;
}

static int rt_fota_read_part(const struct fal_partition *part, int read_pos, rt_uint8_t *buf, rt_uint32_t len)
{
	int rd_len = -1;

	if ((part == RT_NULL) || (buf == RT_NULL) || (len > RT_FOTA_ALGO_BUFF_SIZE))
	{
		goto __partition_read_exit;
	}

	rt_memset(buf, 0x0, len);

	rd_len = fal_partition_read(part, sizeof(rt_fota_part_head) + read_pos, buf, len);

__partition_read_exit:
	return rd_len;
}

static int rt_fota_upgrade(const char *part_name)
{
	int fota_err = RT_FOTA_NO_ERR;
	
	const struct fal_partition *part = RT_NULL;
	rt_fota_part_head_t part_head = RT_NULL;
	
	rt_uint8_t *crypt_buf = RT_NULL;
	
	rt_int32_t fw_raw_pos = 0;
	rt_int32_t fw_raw_len = 0;
	rt_uint32_t total_copy_size = 0;


	if (part_name == RT_NULL)
	{
		LOG_I("Invaild paramenter input!");
		fota_err = RT_FOTA_UPGRADE_CHECK_ERR;
		goto __exit_upgrade;
	}

	part = fal_partition_find(part_name);
	if (part == RT_NULL)
	{		
		LOG_I("Upgrade partition [%s] not found.", part_name);
		fota_err = RT_FOTA_UPGRADE_FIND_ERR;
		goto __exit_upgrade;
	}
	
	/* target partition erase */
	fota_err = rt_fota_erase_app_part();
	if (fota_err != RT_FOTA_NO_ERR)
	{
		goto __exit_upgrade;
	}

	/* rt_fota_erase_app_part() has check fota_part_head vaild already */
	part_head = &fota_part_head;

	crypt_buf = rt_malloc(RT_FOTA_ALGO_BUFF_SIZE);
	if (crypt_buf == RT_NULL)
	{
		LOG_I("Not enough memory for firmware buffer.");
		fota_err = RT_FOTA_UPGRADE_NO_MEM_ERR;
		goto __exit_upgrade;
	}

	LOG_I("Start to copy firmware from %s to %s partition:", part->name, part_head->target_part_name);

	while (fw_raw_pos < part_head->com_size)
	{
        int read_size = RT_FOTA_ALGO_BUFF_SIZE;
        if (fw_raw_pos + RT_FOTA_ALGO_BUFF_SIZE > part_head->com_size) {
            read_size = part_head->com_size - fw_raw_pos;
        }

		fw_raw_len = rt_fota_read_part(part, fw_raw_pos, crypt_buf, read_size);
		if (fw_raw_len < 0)
		{
			LOG_I("read part %s failed.", part->name);
			fota_err = RT_FOTA_UPGRADE_READ_ERR;
			goto __exit_upgrade;
		}		
		fw_raw_pos += fw_raw_len;

		if (rt_fota_write_app_part(total_copy_size, crypt_buf, fw_raw_len) < 0)
		{
			fota_err = RT_FOTA_UPGRADE_WRITE_ERR;
			goto __exit_upgrade;
		}
		
		total_copy_size += fw_raw_len;
		rt_kprintf("=");
	}

    rt_kprintf("\r\n");

	if (total_copy_size != part_head->raw_size)
	{
		LOG_I("total_copy_size check failed.");
		fota_err = RT_FOTA_UPGRADE_SIZECHECK_ERR;
	}

__exit_upgrade:

	if (crypt_buf)
		rt_free(crypt_buf);

	if (fota_err == RT_FOTA_NO_ERR)
	{
    	LOG_I("Upgrade success, total %d bytes.", total_copy_size);
	}
	return fota_err;
}

static int rt_fota_start_bootloader(void)
{
	int res = 0;
	const struct fal_partition *part = RT_NULL;
	rt_uint32_t app_addr = 0;
    fota_flags_t* pflags = fota_flags_get();
    
	part = fal_partition_find(RT_FOTA_BL_PART_NAME);
	if (part == RT_NULL)
	{		
		LOG_E("Partition[%s] not found.", RT_FOTA_BL_PART_NAME);
		res = -1;
		goto __exit_start_application;
	}

	app_addr = part->offset + 0x08000000;
	//判断是否为0x08XXXXXX.
	if (((*(__IO uint32_t *)(app_addr + 4)) & 0xff000000) != 0x08000000)
	{
		LOG_E("Illegal Flash code.");
		res = -2;
		goto __exit_start_application;
	}
	//检查栈顶地址是否合法.
	if (((*(__IO uint32_t *)app_addr) & 0x2ffe0000) != 0x20000000)	
	{
		LOG_E("Illegal Stack code.");
		res = -3;
		goto __exit_start_application;
	}

	LOG_I("Implement bootloader now.");
 
	pflags->uflag.sflag.jump_to_where = FLAG_TO_BL;
	fota_flags_write((uint8_t*)pflags, sizeof(fota_flags_t));
	rt_thread_delay(rt_tick_from_millisecond(20));
    
    __disable_irq();
	Mcu_CoreReset();
    
#if 0
    //HAL_DeInit();  //TODO

    typedef void (*rt_fota_app_func)(void);	
    rt_fota_app_func app_func = RT_NULL;
	//用户代码区第二个字为程序开始地址(复位地址)
	app_func = (rt_fota_app_func)*(__IO uint32_t *)(app_addr + 4);
	/* Configure main stack */
	__set_MSP(*(__IO uint32_t *)app_addr);
	/* jump to application */
	app_func();
#endif
    
__exit_start_application:
	LOG_E("Implement application failed.");
	return res;
}

void rt_fota_thread_entry(void *arg)
{
	//extern int finsh_system_init(void);
	int fota_err = RT_FOTA_NO_ERR;
    fota_flags_t* pflags = RT_NULL;
    const char *src_part = RT_FOTA_DL_PART_NAME;
    
	pflags = fota_flags_get();
	
	/* Partition initialized */
	fota_err = rt_fota_boot_init();
	if (fota_err != RT_FOTA_NO_ERR)
	{
		LOG_I("Partition initialized failed.");
		goto __exit_boot_entry;
	}

	/* Firmware partition verify */
	fota_err = rt_fota_part_fw_verify(src_part, H_CHECK_ALL);
	if (fota_err != RT_FOTA_NO_ERR) {
		goto __exit_boot_entry;
	}

	/* Check upgrade status */
	fota_err = rt_fota_check_upgrade();
	if (fota_err != RT_FOTA_NO_ERR) {
		goto __exit_boot_entry;
	}

	/*pre-set some flags, in case errors occur*/
	pflags->uflag.sflag.jump_to_where = FLAG_TO_APP;
	fota_flags_write((uint8_t*)pflags, sizeof(fota_flags_t));

	/* enter to upgrade mode */
    led_send_upgrade();
    
	/* Implement upgrade, copy firmware partition to app partition */
	fota_err = rt_fota_upgrade(src_part);
	if (fota_err != RT_FOTA_NO_ERR) {
		goto __exit_boot_entry;
	}

__exit_boot_entry:
	pflags->errcode = (uint8_t)(fota_err);

	if (fota_err != RT_FOTA_NO_ERR) {
       
        if (fota_err != RT_FOTA_HEADER_SW_CHECK_ERR) {
            LOG_I("upgrade the bootloader failed.");
        }
	} else {
		/*update bl version*/
		rt_memcpy(pflags->bl_version, fota_part_head.software_version, sizeof(pflags->bl_version));
		LOG_I("upgrade the bootloader success.");
	}

	/*force the jumper to jump to app*/
    pflags->uflag.sflag.force_up = FLAG_NO;
	pflags->uflag.sflag.jump_to_where = FLAG_TO_APP;
	fota_flags_write((uint8_t*)pflags, sizeof(fota_flags_t));
	LOG_I("Mcu core reset...");
    rt_thread_delay(rt_tick_from_millisecond(100));
    /* Reset the device and jump to app */
    Mcu_CoreReset();
    
	//led_send_run();
}


static struct rt_thread fota_thread;
ALIGN(RT_ALIGN_SIZE)
static char fota_thread_stack[RT_FOTA_THREAD_STACK_SIZE];
void rt_fota_init(void)
{
	rt_err_t result = RT_EOK;
    result = rt_thread_init(&fota_thread,
                            "fota",
                            rt_fota_thread_entry, RT_NULL,
                            &fota_thread_stack[0], sizeof(fota_thread_stack),
                            RT_FOTA_THREAD_PRIORITY, 10);

    if (result == RT_EOK)
    	rt_thread_startup(&fota_thread);
    else
    	LOG_I("fota thread create failed.");
}

void rt_fota(rt_uint8_t argc, char **argv)
{
	char put_buf[24];
	char part_name[2][FAL_DEV_NAME_MAX] = 
    {
        {RT_FOTA_DL_PART_NAME}, 
        {RT_FOTA_BK_PART_NAME}
    };
		
	const char* help_info[] =
    {
            [0]     = "fota probe                       		- probe UBL file of partiton",
            [1]     = "fota show partition addr size    		- show 'size' bytes starting at 'addr'",
            [2]     = "fota clone des_part src_part     		- clone src partition to des partiton",
            [3]     = "fota jump                        		- execute bootloader program",
            [4]     = "fota flags                       		- printf flags in eeprom",
            [5]     = "fota flags name value            	    - write  flags to eeprom",
            [6]     = "fota upgrade [download/backup] [app/bl]  - upgrade firmware",
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
		if (!rt_strcmp(operator, "probe"))
		{
	    	for (int i = 0; i < 2; i++)
	    	{
	    		if (rt_fota_part_fw_verify(&part_name[i][0], H_NO_CHECK_TARGET_NAME) == RT_FOTA_NO_ERR)
		    	{
		    		rt_kprintf("===== RBL of %s partition =====\n", &part_name[i][0]);
		    		rt_kprintf("| target part name   | %*.s |\n", 11, fota_part_head.target_part_name);

					rt_memset(put_buf, 0x0, sizeof(put_buf));
					if ((fota_part_head.fota_algo & RT_FOTA_CRYPT_STAT_MASK) == RT_FOTA_CRYPT_ALGO_AES256)
					{
						rt_strncpy(put_buf, " AES", 4);
					}
					else if ((fota_part_head.fota_algo & RT_FOTA_CRYPT_STAT_MASK) == RT_FOTA_CRYPT_ALGO_XOR)
					{
						rt_strncpy(put_buf, " XOR", 4);
					}
                    else
                    {
                        rt_strncpy(put_buf, "NONE", 4);
                    }

					if ((fota_part_head.fota_algo & RT_FOTA_CMPRS_STAT_MASK) == RT_FOTA_CMPRS_ALGO_GZIP)
					{
						rt_strncpy(&put_buf[rt_strlen(put_buf)], " && GLZ", 7);
					}
					else if ((fota_part_head.fota_algo & RT_FOTA_CMPRS_STAT_MASK) == RT_FOTA_CMPRS_ALGO_QUICKLZ)
					{
						rt_strncpy(&put_buf[rt_strlen(put_buf)], " && QLZ", 7);
					}
					else if ((fota_part_head.fota_algo & RT_FOTA_CMPRS_STAT_MASK) == RT_FOTA_CMPRS_ALGO_FASTLZ)
					{
						rt_strncpy(&put_buf[rt_strlen(put_buf)], " && FLZ", 7);
					}

					if (rt_strlen(put_buf) <= 0)
					{
						rt_strncpy(put_buf, "None", 4);
					}
					rt_kprintf("| Algorithm mode     | %*.s |\n", 11, put_buf);
					rt_kprintf("| Software version   | %*.s |\n", 11, fota_part_head.software_version);
					rt_kprintf("| Hardware version   | %*.s |\n", 11, fota_part_head.hardware_version);
					rt_kprintf("| Code raw size      | %11d |\n", fota_part_head.raw_size);
	                rt_kprintf("| Code package size  | %11d |\n", fota_part_head.com_size);
					rt_kprintf("| Build Timestamp    | %11d |\n", *((rt_uint32_t *)(&fota_part_head.fm_time[2])));			
                    //rt_kprintf("\r\n");
		    	}
	    	} 			
		}
       	else if (!rt_strcmp(operator, "show"))
       	{
       		const struct fal_partition *part;
       		const char *part_name = argv[2];
			
			rt_uint32_t addr = strtol(argv[3], NULL, 0);
			rt_uint32_t size = strtol(argv[4], NULL, 0);
			rt_uint8_t buf[16];
			
			
			part = fal_partition_find(part_name);
			if (part != RT_NULL)
			{
				while (size > 16)
				{
					fal_partition_read(part, addr, buf, 16);					
					
					rt_kprintf("%08X: ", addr);
					for (int i = 0; i < 16; i++)
					{
						rt_kprintf("%02X ", buf[i]);
					}
					rt_kprintf("\n");

					size -= 16;
					addr += 16;
				}

				fal_partition_read(part, addr, buf, size);
				rt_kprintf("%08X: ", addr);
				for (int i = 0; i < size; i++)
				{
					rt_kprintf("%02X ", buf[i]);
				}
				rt_kprintf("\n");
			}       
			else
			{
				rt_kprintf("%s partition is not exist!\n", part_name);
			}
       	}
		else if (!rt_strcmp(operator, "clone"))
		{
       		const char *dst_part_name = argv[2];
			const char *src_part_name = argv[3];
			const struct fal_partition *dst_part;
			const struct fal_partition *src_part;

			dst_part = fal_partition_find(dst_part_name);
			src_part = fal_partition_find(src_part_name);
			if (dst_part == RT_NULL || src_part == RT_NULL)
			{
				if (dst_part == RT_NULL)
					rt_kprintf("%s partition is not exist!\n", dst_part_name);

				if (src_part == RT_NULL)
					rt_kprintf("%s partition is not exist!\n", src_part_name);
			}
			else
			{
				rt_kprintf("Clone %s partition to %s partition:\n", src_part_name, dst_part_name);
				if (fal_partition_erase(dst_part, 0, dst_part->len) >= 0)
				{
					int clone_pos = 0;
					int clone_len = 0, clone_tol_len;
					rt_uint8_t *buf = rt_malloc(4096);

					if (dst_part->len < src_part->len)
						clone_tol_len = dst_part->len;
					else
						clone_tol_len = src_part->len;
					
					while ((clone_pos < clone_tol_len) && (buf != RT_NULL))
					{
						clone_len = fal_partition_read(src_part, clone_pos, buf, 4096);
						if (clone_len < 0)
						{
							rt_kprintf("\nread %s partition failed, clone stop!\n", src_part_name);
							break;
						}
						
						if (fal_partition_write(dst_part, clone_pos, buf, clone_len) < 0)
						{
							rt_kprintf("\nwrite %s partition failed, clone stop!\n", dst_part_name);
							break;
						}

						rt_kprintf("#");
						clone_pos += clone_len;
					}
					
					if (clone_pos >= clone_tol_len)
						rt_kprintf("\nClone partition success, total %d bytes!\n", clone_tol_len);	
					else
						rt_kprintf("\nClone partition failed!\n");

					if (buf)
						rt_free(buf);
				}
			}
		}
		else if (!rt_strcmp(operator, "jump"))
		{
			rt_fota_start_bootloader();
		}
		else if (!rt_strcmp(operator, "flags"))    // TODO 实现写flag的接口
		{
			fota_flags_t* pflags = fota_flags_get();
            
			fota_flags_read((uint8_t*)(pflags), sizeof(fota_flags_t));

			rt_kprintf("jump_to_where    : %s \n", pflags->uflag.sflag.jump_to_where == FLAG_TO_BL ? "bootloader" : "app");
            rt_kprintf("update_from      : %s \n", pflags->uflag.sflag.update_from == FLAG_FROM_DL ? RT_FOTA_DL_PART_NAME : RT_FOTA_BK_PART_NAME);
            rt_kprintf("force_update_app : %s \n", pflags->uflag.sflag.force_up == FLAG_YES ? "YES" : "NO");
            rt_kprintf("initialized      : %s \n", pflags->initialized == FLAG_INIT ? "YES" : "NO");
            rt_kprintf("errcode          : %d \n", pflags->errcode);
            rt_kprintf("tobl_cnt         : %d \n", pflags->tobl_cnt);
			rt_kprintf("app_version      : %s \n", pflags->app_version);
			rt_kprintf("bl_version       : %s \n", pflags->bl_version);
            rt_kprintf("jmp_version      : %d \n", pflags->jumper_version);
		}
		else if (!rt_strcmp(operator, "upgrade"))
		{
			if (argc == 4) 
			{
	            const char *src_part = argv[2];
	            const char *tar_part = argv[3];

	            fota_flags_t* pflags = fota_flags_get();
	            if (!rt_strcmp(tar_part, "app")) 
	            {
					//pflags->uflag.sflag.force_up = FLAG_YES;   // TODO  for test
	                if (!rt_strcmp(src_part, "download"))
	                {       
	                    pflags->uflag.sflag.jump_to_where = FLAG_TO_BL;
	                    pflags->uflag.sflag.update_from = FLAG_FROM_DL;
	                } else if (!rt_strcmp(src_part, "backup")) {
	                    pflags->uflag.sflag.jump_to_where = FLAG_TO_BL;
	                    pflags->uflag.sflag.update_from = FLAG_FROM_BK;
	                }
	                else {
	                    rt_kprintf("source partition [%s] DONOT surport.\n", src_part);
	                    return;
	                }
	                fota_flags_write((uint8_t*)pflags, sizeof(fota_flags_t));
	                /* wait some time for terminal response finish */
	                rt_thread_delay(rt_tick_from_millisecond(100));
	                /* Reset the device and jump to bootloader to upgrade firmware */
	                Mcu_CoreReset();
	            }
	            else if (!rt_strcmp(tar_part, "bl")) {
	                extern void rt_fota_init(void);
	                rt_fota_init();
                    while(1) {
                        /*hang up self thread, wait for upgrade finish and reset the mcu*/
                        rt_thread_delay(rt_tick_from_millisecond(1000));
                    }
	            }
	            else {
	                rt_kprintf("target partition [%s] DONOT surport.\n", tar_part);
	            }
			}
			else {
				rt_kprintf("fota upgrade [download/backup] [app/bl]\n");
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
MSH_CMD_EXPORT_ALIAS(rt_fota, fota, fota functions);

