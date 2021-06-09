#include <rtthread.h>
#include <dfs_posix.h>
#include <fal.h>
#include "rt_fota_cfg.h"
#include "rt_fota_flag.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME          "tftp_ota"
#ifdef OTA_DOWNLOADER_DEBUG
#define DBG_LEVEL                 DBG_LOG
#else
#define DBG_LEVEL                 DBG_INFO
#endif 
#define DBG_COLOR
#include <rtdbg.h>


#define READ_BUFF_LEN (4*1024)
#define DEFAULT_DOWNLOAD_PART RT_FOTA_DL_PART_NAME
#define FW_PATH "/update/rtthread_app_v2.0.0.rbl"


static void print_progress(size_t cur_size, size_t total_size)
{
    static unsigned char progress_sign[100 + 1];
    uint8_t i, per = cur_size * 100 / total_size;

    if (per > 100)
    {
        per = 100;
    }

    for (i = 0; i < 100; i++)
    {
        if (i < per)
        {
            progress_sign[i] = '=';
        }
        else if (per == i)
        {
            progress_sign[i] = '>';
        }
        else
        {
            progress_sign[i] = ' ';
        }
    }

    progress_sign[sizeof(progress_sign) - 1] = '\0';

    LOG_I("\033[2A");
    LOG_I("Download: [%s] %d%%", progress_sign, per);
}



static int copy_file_to_part(const char* part)
{
    int ret = -RT_ERROR;
    int fd = -1;
    int file_size, length, total_length = 0;
    rt_uint8_t *buffer_read = RT_NULL;
    const struct fal_partition * dl_part = RT_NULL;

    /*get file size*/
    struct stat stat;
    if (dfs_file_stat(FW_PATH, &stat) < 0) {
        LOG_E("Firmware download failed! read file [%s] size error!", FW_PATH);
        ret = -RT_ERROR;
        goto __exit;   
    }
    file_size = stat.st_size;

    if ((dl_part = fal_partition_find(part)) == RT_NULL)
    {
        LOG_E("Firmware download failed! Partition (%s) find error!", part);
        ret = -RT_ERROR;
        goto __exit;
    }

    LOG_I("Start erase flash (%s) partition!", dl_part->name);

    if (fal_partition_erase(dl_part, 0, file_size) < 0)
    {
        LOG_E("Firmware download failed! Partition (%s) erase error!", dl_part->name);
        goto __exit;
    }
    LOG_I("Erase flash (%s) partition success!", dl_part->name);


    buffer_read = rt_malloc(READ_BUFF_LEN);
    if (buffer_read == RT_NULL) {
        LOG_E("Firmware download failed! No memory for transfer!");
        ret = -RT_ERROR;
        goto __exit;
    }
    rt_memset(buffer_read, 0x00, READ_BUFF_LEN);

    fd = open(FW_PATH, O_RDONLY, 0);
    if (fd < 0) {
        LOG_E("Firmware download failed! Cannot open file [%s]!", FW_PATH);
        ret = -RT_ERROR;
        goto __exit;
    }

    do
    {
        length = read(fd, buffer_read, file_size - total_length > READ_BUFF_LEN ?
                            READ_BUFF_LEN : file_size - total_length);   
        if (length >= 0) // > 0
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
            LOG_E("Firmware download failed! read file err (%d)!", length);
            ret = -RT_ERROR;
            goto __exit;
        }

    } while(total_length != file_size);

    ret = RT_EOK;

    if (total_length == file_size)
    {
        LOG_I("firmware copy to [%s] success.", part);
        // LOG_I("System now will restart...");

        rt_thread_delay(rt_tick_from_millisecond(5));

        /* Reset the device, Start new firmware */
        // extern void rt_hw_cpu_reset(void);
        // rt_hw_cpu_reset();
    }

__exit:
    if (fd > 0) {
        close(fd);
    }
    if (buffer_read != RT_NULL) {
        rt_free(buffer_read);
    }

    return ret;
}


#if defined(RT_USING_FINSH)
#include <finsh.h>

static void tftp_ota(uint8_t argc, char **argv)
{
    char* recv_partition = DEFAULT_DOWNLOAD_PART;

    #define PRINT_INFO \
    { \
        rt_kprintf("tftp_ota -p <part>                 --copy files to <download/default> part\n"); \
        rt_kprintf("tftp_ota -u <src_part> <tar_part>  --update from <download/default> to <app/bl>\n"); \
        rt_kprintf("tftp_ota -h                        --help information  \n"); \
    }

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
                if (!rt_strcmp(part, "download") || !rt_strcmp(part, "default")) {
                    recv_partition = argv[2];
                }
                else {
                    rt_kprintf("tftp_ota -p <part>                 --download files to <download/default> part\n");
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
                    } else if (!rt_strcmp(src_part, "default")) {
                        pflags->uflag.sflag.jump_to_where = FLAG_TO_BL;
                        pflags->uflag.sflag.update_from = FLAG_FROM_DF;
                    }
                    else {
                        rt_kprintf("source partition [%s] DONOT surport.\n", src_part);
                        return;
                    }
                    ee_write((uint8_t*)pflags, sizeof(fota_flags_t));
                    /* wait some time for terminal response finish */
                    rt_thread_delay(rt_tick_from_millisecond(200));
                    /* Reset the device, Start new firmware */
                    extern void rt_hw_cpu_reset(void);
                    rt_hw_cpu_reset();
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

    copy_file_to_part(recv_partition);

}
MSH_CMD_EXPORT(tftp_ota, start tftp ota.);


#endif /* defined(RT_USING_FINSH) */
