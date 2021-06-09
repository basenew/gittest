/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-30     armink       the first version
 * 2018-08-27     Murphy       update log
 */

#include <rtthread.h>
#include <stdio.h>
#include <stdbool.h>
#include <finsh.h>
#include <fal.h>
#include <ymodem.h>
#include "rt_fota_cfg.h"
#include "rt_fota_flag.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME               "ymodem"
#ifdef OTA_DOWNLOADER_DEBUG
#define DBG_LEVEL                      DBG_LOG
#else
#define DBG_LEVEL                      DBG_INFO
#endif
#define DBG_COLOR
#include <rtdbg.h>

#ifdef PKG_USING_YMODEM_OTA

#define DEFAULT_DOWNLOAD_PART RT_FOTA_DL_PART_NAME

static char* recv_partition = DEFAULT_DOWNLOAD_PART;
static size_t update_file_total_size, update_file_cur_size;
static const struct fal_partition * dl_part = RT_NULL;

static enum rym_code ymodem_on_begin(struct rym_ctx *ctx, rt_uint8_t *buf, rt_size_t len)
{
    char *file_name, *file_size;

    /* calculate and store file size */
    file_name = (char *)&buf[0];
    file_size = (char *)&buf[rt_strlen(file_name) + 1];
    update_file_total_size = atol(file_size);
    rt_kprintf("Ymodem file_size:%d\n", update_file_total_size);

    update_file_cur_size = 0;

    /* Get download partition information and erase download partition data */
    if ((dl_part = fal_partition_find(recv_partition)) == RT_NULL)
    {
        LOG_E("Firmware download failed! Partition (%s) find error!", recv_partition);
        return RYM_CODE_CAN;
    }

    if (update_file_total_size > dl_part->len)
    {
        LOG_E("Firmware is too large! File size (%d), '%s' partition size (%d)", update_file_total_size, recv_partition, dl_part->len);
        return RYM_CODE_CAN;
    }

    LOG_I("Start erase. Size (%d)", update_file_total_size);

    /* erase DL section */
    if (fal_partition_erase(dl_part, 0, update_file_total_size) < 0)
    {
        LOG_E("Firmware download failed! Partition (%s) erase error!", dl_part->name);
        return RYM_CODE_CAN;
    }

    return RYM_CODE_ACK;
}

static enum rym_code ymodem_on_data(struct rym_ctx *ctx, rt_uint8_t *buf, rt_size_t len)
{
    /* write data of application to DL partition  */
    if (fal_partition_write(dl_part, update_file_cur_size, buf, len) < 0)
    {
        LOG_E("Firmware download failed! Partition (%s) write data error!", dl_part->name);
        return RYM_CODE_CAN;
    }

    update_file_cur_size += len;

    return RYM_CODE_ACK;
}

void ymodem_ota(uint8_t argc, char **argv)
{
    #define PRINT_INFO \
    { \
        rt_kprintf("ymodem_ota -p <part>                 --download files to <download/backup> part\n"); \
        rt_kprintf("ymodem_ota -u <src_part> <tar_part>  --update from <download/backup> to <app/bl>\n"); \
        rt_kprintf("ymodem_ota -h                        --help information  \n"); \
    }

    struct rym_ctx rctx;

    if (argc < 2)
    {
       PRINT_INFO;
       return;
    }
    else 
    {
        const char *operator = argv[1];
        if (!rt_strcmp(operator, "-p")) {
            if (argc == 2) {
                recv_partition = DEFAULT_DOWNLOAD_PART;
            }
            else {
                const char *part = argv[2];
                if (!rt_strcmp(part, "download") || !rt_strcmp(part, "backup")) {
                    recv_partition = argv[2];
                }
                else {
                    rt_kprintf("ymodem_ota -p <part>                 --download files to <download/default> part\n");
                    return;
                }
            }
        } 
        else if (!rt_strcmp(operator, "-u")) {
            if (argc == 4) 
            {
                const char *src_part = argv[2];
                const char *tar_part = argv[3];

                fota_flags_t* pflags = fota_flags_get();
                if (!rt_strcmp(tar_part, "app")) 
                {
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
                    rt_thread_delay(rt_tick_from_millisecond(200));
                    /* Reset the device, Start new firmware */
                    Mcu_CoreReset();
                }
                else if (!rt_strcmp(tar_part, "bl")) {
                    extern void rt_fota_init(void);
                    rt_fota_init();
                }
                else {
                    rt_kprintf("target partition [%s] DONOT surport.\n", tar_part);
                }
            }
            else {
                rt_kprintf("ymodem_ota -u <src_part> <tar_part>  --update from <download/default> to <app/bl>\n"); \
            }
            return;
        } 
        else if (!rt_strcmp(operator, "-h")) {
            PRINT_INFO;
            return;
        }
        else {
           PRINT_INFO;
           return;
        }
    }

    rt_kprintf("Save firmware on [%s] partition...\n", recv_partition);
    rt_kprintf("Warning: Ymodem has started! This operator will not recovery.\n");
    rt_kprintf("Please select the ota firmware file and use Ymodem to send.\n");

    if (!rym_recv_on_device(&rctx, rt_console_get_device(), RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                            ymodem_on_begin, ymodem_on_data, NULL, RT_TICK_PER_SECOND))
    {
        rt_kprintf("Download firmware to partition [%s] success.\n", recv_partition);
        /* wait some time for terminal response finish */
        rt_thread_delay(rt_tick_from_millisecond(200));
    }
    else
    {
        // 没有下载成功。。。。 TODO

        /* wait some time for terminal response finish */
        rt_thread_delay(RT_TICK_PER_SECOND);
        rt_kprintf("Update firmware fail.\n");
    }

    return;
}
/**
 * msh />ymodem_ota
*/
MSH_CMD_EXPORT(ymodem_ota, Use Y-MODEM to download the firmware);

#endif /* PKG_USING_YMODEM_OTA */
