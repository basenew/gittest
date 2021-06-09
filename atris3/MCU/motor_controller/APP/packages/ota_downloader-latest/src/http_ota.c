/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-03-22     Murphy       the first version
*  2020-06-01     wuxiaofeng   modify for ubtech
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <rtthread.h>
#include <finsh.h>

#include "webclient.h"
#include <fal.h>

#include "rt_fota_cfg.h"
#include "rt_fota_flag.h"
#include "ota_downloader.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME          "http_ota"
#ifdef OTA_DOWNLOADER_DEBUG
#define DBG_LEVEL                 DBG_LOG
#else
#define DBG_LEVEL                 DBG_INFO
#endif 
#define DBG_COLOR
#include <rtdbg.h>

#ifdef PKG_USING_HTTP_OTA

#define HTTP_OTA_BUFF_LEN         4096
#define GET_HEADER_BUFSZ          1024
#define GET_RESP_BUFSZ            1024
#define HTTP_OTA_DL_DELAY         (10 * RT_TICK_PER_SECOND)

#define HTTP_OTA_URL              PKG_HTTP_OTA_URL

#ifndef RT_FOTA_DL_PART_NAME
#define RT_FOTA_DL_PART_NAME    "download"
#endif

#ifndef RT_FOTA_BK_PART_NAME
#define RT_FOTA_BK_PART_NAME    "backup"
#endif

#ifndef RT_FOTA_APP_PART_NAME
#define RT_FOTA_APP_PART_NAME   "app"
#endif

#ifndef RT_FOTA_BL_PART_NAME
#define RT_FOTA_BL_PART_NAME   "bl"
#endif

static void print_progress(size_t cur_size, size_t total_size)
{
    int percnt = cur_size * 100 / total_size;
    if (percnt > 100) percnt = 100;
    rt_kprintf("Downloading: %d%%\033[1A\r\n", percnt);
}


static int http_ota_fw_download(const char* tar_partname, const char* uri)
{
    int ret = -RT_ERROR, resp_status = -1;
    int file_size = 0, length, total_length = 0;
    rt_uint8_t *buffer_read = RT_NULL;
    struct webclient_session* session = RT_NULL;
    const struct fal_partition * dl_part = RT_NULL;

    /* create webclient session and set header response size */
    session = webclient_session_create(GET_HEADER_BUFSZ);
    if (!session)
    {
        LOG_E("open uri failed.");
        ret = -RT_ERROR;
        goto __exit;
    }

    /* send GET request by default header */
    if ((resp_status = webclient_get(session, uri)) != 200)
    {
        LOG_E("webclient GET request failed, response(%d) error.", resp_status);
        ret = -RT_ERROR;
        goto __exit;
    }

    file_size = webclient_content_length_get(session);
    rt_kprintf("http file_size:%d\n",file_size);

    if (file_size == 0)
    {
        LOG_E("Request file size is 0!");
        ret = -RT_ERROR;
        goto __exit;
    }
    else if (file_size < 0)
    {
        LOG_E("webclient GET request type is chunked.");
        ret = -RT_ERROR;
        goto __exit;
    }

    /* Get download partition information and erase download partition data */
    if ((dl_part = fal_partition_find(tar_partname)) == RT_NULL)
    {
        LOG_E("Firmware download failed! Partition (%s) find error!", tar_partname);
        ret = -RT_ERROR;
        goto __exit;
    }

    LOG_I("Start erase flash (%s) partition!", dl_part->name);

    if (fal_partition_erase(dl_part, 0, file_size) < 0)
    {
        LOG_E("Firmware download failed! Partition (%s) erase error!", dl_part->name);
        ret = -RT_ERROR;
        goto __exit;
    }
    LOG_I("Erase flash (%s) partition success!", dl_part->name);

    buffer_read = web_malloc(HTTP_OTA_BUFF_LEN);
    if (buffer_read == RT_NULL)
    {
        LOG_E("No memory for http ota!");
        ret = -RT_ERROR;
        goto __exit;
    }
    rt_memset(buffer_read, 0x00, HTTP_OTA_BUFF_LEN);

    LOG_I("OTA file size is (%d)", file_size);

    LOG_I("Start download firmware to Partition(%s)", dl_part->name);
    do
    {
        length = webclient_read(session, buffer_read, file_size - total_length > HTTP_OTA_BUFF_LEN ?
                            HTTP_OTA_BUFF_LEN : file_size - total_length);   
        if (length > 0)
        {
            /* Write the data to the corresponding partition address */
            if (fal_partition_write(dl_part, total_length, buffer_read, length) < 0)
            {
                LOG_E("Firmware download failed! Partition (%s) write data error!", dl_part->name);
                ret = -RT_ERROR;
                goto __exit;
            }
            total_length += length;

            print_progress(total_length, file_size);
        }
        else
        {
            LOG_E("Exit: server return err (%d)!", length);
            ret = -RT_ERROR;
            goto __exit;
        }

    } while(total_length != file_size);

    

    if (total_length == file_size)
    {
        ret = RT_EOK;
        LOG_I("Download firmware to Partition(%s) success.", dl_part->name);
    }
    else {
        ret = -RT_ERROR;
    }

__exit:
    if (session != RT_NULL)
        webclient_close(session);
    if (buffer_read != RT_NULL)
        web_free(buffer_read);

    return ret;
}

void http_ota(uint8_t argc, char **argv)
{
    #define PRINT_INFO \
    { \
        rt_kprintf("http_ota [url] [download/backup] [app/bl] - from [url] download firmware to part[download/backup] then ugrade[app/bl]\n"); \
        rt_kprintf("http_ota [url] [download/backup]          - from [url] download firmware to part[download/backup]\n"); \
    }

    if (argc < 3)
    {
        PRINT_INFO;
    }
    else
    {
        char *tar_partname = RT_NULL;
        char *upgrade_part = RT_NULL;
        char *src_url = RT_NULL;
        if (argc == 3)
        {
            src_url = argv[1];
            tar_partname = argv[2];
            if (rt_strcmp(tar_partname, RT_FOTA_DL_PART_NAME) && rt_strcmp(tar_partname, RT_FOTA_BK_PART_NAME))
            {
                PRINT_INFO;
            }
            else {
                http_ota_fw_download(tar_partname, src_url);
            }
        }
        else if (argc == 4) 
        {
            src_url = argv[1];
            tar_partname = argv[2];
            upgrade_part = argv[3];
            if (rt_strcmp(tar_partname, RT_FOTA_DL_PART_NAME) && rt_strcmp(tar_partname, RT_FOTA_BK_PART_NAME))
            {
                PRINT_INFO;
            }
            else if (rt_strcmp(upgrade_part, RT_FOTA_APP_PART_NAME) && rt_strcmp(upgrade_part, RT_FOTA_BL_PART_NAME))
            {
                PRINT_INFO;
            }
            else
            {
                if (http_ota_fw_download(tar_partname, src_url) == RT_EOK)
                {
                    extern void rt_fota(rt_uint8_t argc, char **argv);
                    static char* fota_cmd[1][4] = {
                        [0] = {"fota", "upgrade", "download/backup", "app/bl"},
                    }; 

                    fota_cmd[0][2] = tar_partname;
                    fota_cmd[0][3] = upgrade_part;
                    rt_fota(4, fota_cmd[0]);
                }
            }
        }
        else
        {
            PRINT_INFO;
        }
    }
}
MSH_CMD_EXPORT(http_ota, Use HTTP to download the firmware);




static void upbl_test(uint8_t argc, char **argv)
{
    static char* test_cmd[1][4] = {
        [0] = {"http_ota", "http://10.20.18.2/hfs/bootloader.rbl", "download", "bl"},
    }; 
    int cnt = 100;
    while (cnt --)
    {
        http_ota(4, test_cmd[0]);
        rt_thread_mdelay(3000);
    }
}
MSH_CMD_EXPORT(upbl_test, test upgrade bootloader);


#endif /* PKG_USING_HTTP_OTA */
