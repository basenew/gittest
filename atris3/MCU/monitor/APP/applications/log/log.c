/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : log.c
 * Brief      : log manager

 * Change Logs
 * Date           Author        Version       Notes
 * 2019-07-23     wuxiaofeng    v1.0          
 * 2020-06-18     wuxiaofeng    v1.1          update     
 */
#include "rtthread.h"
#include "app_cfg.h"
#include <drivers/rtc.h>
#include <dfs_posix.h>
#include "hw_wdg.h"
#include "log.h"
#include "spi_flash_init.h"


#ifdef ULOG_TIME_USING_TIMESTAMP
#include <sys/time.h>
#endif


#define SDEBUG(...)      rt_kprintf(__VA_ARGS__)

#ifndef TASK_STACK_SIZE_LOG
#define TASK_STACK_SIZE_LOG   4096
#endif

#ifndef TASK_PRIORITY_LOG
#define TASK_PRIORITY_LOG     25
#endif


#define LOG_DIR     "/logs"
#define LOG_ASYNC_OUTPUT_BUF_SIZE (1024*4)
#define LOG_LINE_BUF_SIZE (RT_CONSOLEBUF_SIZE)
#define LOG_ASYNC_OUTPUT_STORE_LINES  ((LOG_ASYNC_OUTPUT_BUF_SIZE) * 3 / 2 / (LOG_LINE_BUF_SIZE))
#define LOG_CLEANUP_PERIOD (60*1000)

typedef struct
{
    uint8_t isinit;
    rt_rbb_t async_rbb;
    struct rt_semaphore async_notice;
} log_t;

typedef struct
{
    char* buf;
    int  len;
} log_msg_t;

static log_t log_obj = 
{
    .isinit = 0,
    .async_rbb = RT_NULL,
};

/*02-01-2020-06 day-mouth-year-idx*/
static char g_today[20] = {0};



static void feed_watchdog(void) 
{
    hw_wdg_feed(); 
}

/*01-01-2020-06 -> 2020010106*/
static uint32_t exchange_filename(char name[])
{
    char no[2+1] = {0};
    char day[2+1] = {0};
    char mon[2+1] = {0};
    char year[4+1] = {0};
    
    rt_memcpy(day, &name[0], 2);
    rt_memcpy(mon, &name[3], 2);
    rt_memcpy(year,&name[6], 4);
    rt_memcpy(no,  &name[11], 2);

    return atoi(year)*1000000 + atoi(mon)*10000 + atoi(day)*100 + atoi(no);
}

static int qsort_cmp(const void *a, const void *b)
{   //min-->max
    return *(int*)a - *(int*)b;
}

#define NO_RTC_DATETIME 0
#define LOG_FILE_MAX 31 //31 day
#define LOG_FILE_SIZE (100*1024)
static uint32_t g_file_array[LOG_FILE_MAX] = {0};
static uint32_t g_file_cnts = 0;
static uint32_t g_file_idx = 0;
static uint32_t g_today_file_size = 0;

static int get_files_info()
{
    int file_cur = 0;
    int file_cnt = 0;
    DIR *dir = RT_NULL;
    struct dirent* dirent;
    struct stat s;
    static char fullpath[32] = {0};

    rt_memset(g_file_array, 0, sizeof(g_file_array));
    feed_watchdog();
    dir = opendir(LOG_DIR);
    if (dir != RT_NULL)
    {
        do 
        {
            feed_watchdog();
            dirent = readdir(dir);
            if (dirent == RT_NULL) {
                break;
            }
            
            rt_memset(&s, 0, sizeof(struct stat));
            // build full path for each file 
            rt_memset(fullpath, 0, sizeof(fullpath));
            rt_sprintf(fullpath, "%s/%s", LOG_DIR, dirent->d_name);
            stat(fullpath, &s);
            if ( s.st_mode & S_IFDIR ) {
                ;
            }
            else {
                file_cur = exchange_filename(dirent->d_name);
                if (file_cnt < LOG_FILE_MAX) {
                    g_file_array[file_cnt++] = file_cur;
                }
                else {
                    /*just delete extra files*/
                    rt_memset(fullpath, 0, sizeof(fullpath));
                    rt_sprintf(fullpath, "%s/%s", LOG_DIR, dirent->d_name);
                    //rt_sprintf(fullpath, "%s/%02d-%02d-%04d-%02d", LOG_DIR, file_cur%1000000/100, file_cur/10000, file_cur/1000000, file_cur%10000000);
                    feed_watchdog();
                    if (unlink(fullpath) == 0) {
                        SDEBUG("delete %s suceess.\n", fullpath);
                    } else {
                        SDEBUG("delete %s fail.\n", fullpath);
                    }
                }
            }
        } while (dirent != RT_NULL);
        
        feed_watchdog();
        closedir(dir);
    }
    else {
        return -1;
    }
    
    g_file_cnts = file_cnt;
    if (g_file_cnts >= LOG_FILE_MAX) {
        g_file_cnts = LOG_FILE_MAX;
    }
    
    if (g_file_cnts > 0) {
        feed_watchdog();
        qsort(g_file_array, g_file_cnts, sizeof(g_file_array[0]), qsort_cmp);
        feed_watchdog();
        file_cur = g_file_array[g_file_cnts-1];
        g_file_idx = file_cur%100;
        rt_sprintf(&g_today[11], "%02d", g_file_idx);
        
        rt_memset(&s, 0, sizeof(struct stat));
        rt_memset(fullpath, 0, sizeof(fullpath));
        rt_sprintf(fullpath, "%s/%02d-%02d-%04d-%02d", LOG_DIR, file_cur%10000/100, file_cur%1000000/10000, file_cur/1000000, file_cur%100);
        stat(fullpath, &s);
        g_today_file_size = s.st_size;  
    }
    else {
        g_today_file_size = 0;
    }
    return 0;
}

static int fs_write(char* path, char *log, int len)
{
    int fd = 0;
    int length = 0;
    
    feed_watchdog();
    fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0);
    if (fd < 0)
    {
        SDEBUG("log cannot open dir!\n");
        return -1;
    }
    
    feed_watchdog();
    length = write(fd, log, len);
    if (length != len)
    {
        SDEBUG("error occured when write log.\n");
        close(fd);
        return -2;
    }
    feed_watchdog();
    close(fd);   
    return 0;
}

static void log_write(char *log, int len)
{
    char path[sizeof(LOG_DIR)+sizeof(g_today)+1] = {0};

    if (log == RT_NULL) return;

    g_today_file_size += len;
    if (g_today_file_size >= LOG_FILE_SIZE) 
    {
        if (g_file_idx >= LOG_FILE_MAX - 1) {
            g_file_idx = 0;
        } else {
            g_file_idx ++; 
        }
        
        if (g_file_cnts == LOG_FILE_MAX) {
            uint32_t file_min = g_file_array[g_file_idx];
            rt_memset(path, 0, sizeof(path));
            rt_sprintf(path, "%s/%02d-%02d-%04d-%02d", LOG_DIR, file_min%10000/100, file_min%1000000/10000, file_min/1000000, file_min%100);
            feed_watchdog();
            if (unlink(path) == 0) {
                SDEBUG("delete %s suceess.\n", path);
                g_today_file_size = 0;
                rt_sprintf(&g_today[11], "%02d", g_file_idx);
                g_file_array[g_file_idx] = exchange_filename(g_today);

            } else {
                g_file_idx = (g_file_idx == 0 ? LOG_FILE_MAX - 1 : g_file_idx - 1);
                SDEBUG("delete %s fail.\n", path);
            }
            rt_memset(path, 0, sizeof(path));
        }
        else {
            rt_sprintf(&g_today[11], "%02d", g_file_idx);
            g_file_array[g_file_idx] = exchange_filename(g_today);
            if (g_file_cnts >= LOG_FILE_MAX) {
                g_file_cnts = LOG_FILE_MAX;
            } else {
                g_file_cnts ++;
            }
        }
    }

    rt_sprintf(path, "%s/%s", LOG_DIR, g_today);
    if(fs_write(path, log, len) < 0)
    {
        //just put out a tip.
        rt_kprintf("fs_write failed.\n");
    }
}


int log_notify_msg(const char* buf, int len)
{
    rt_rbb_blk_t log_blk;

    /* allocate log frame */
    log_blk = rt_rbb_blk_alloc(log_obj.async_rbb, RT_ALIGN(len, RT_ALIGN_SIZE));
    if (log_blk)
    {
        /* copy log data */
        rt_memcpy(log_blk->buf, buf, len);
        log_blk->size = (len > LOG_LINE_BUF_SIZE) ? LOG_LINE_BUF_SIZE : len;
        
        /* put the block */
        rt_rbb_blk_put(log_blk);
        /* send a notice */
        return rt_sem_release(&log_obj.async_notice);
    }
    else
    {
        static rt_bool_t already_output = RT_FALSE;
        if (already_output == RT_FALSE)
        {
            SDEBUG("Warning: There is no enough buffer for saving log,"
                    " please increase the LOG_ASYNC_OUTPUT_BUF_SIZE option.\n");
            already_output = RT_TRUE;
        }
        return -1;
    }
}

static rt_err_t log_async_waiting_log(rt_int32_t time)
{
    rt_sem_control(&log_obj.async_notice, RT_IPC_CMD_RESET, RT_NULL);
    return rt_sem_take(&log_obj.async_notice, time);
}

static void log_async_output(void)
{
    rt_rbb_blk_t log_blk;
    char* pbuf = RT_NULL;

    while ((log_blk = rt_rbb_blk_get(log_obj.async_rbb)) != NULL)
    {
        feed_watchdog();
        pbuf = (char*) log_blk->buf;
        log_write(pbuf, log_blk->size);
        rt_rbb_blk_free(log_obj.async_rbb, log_blk);
    }
}

static void task_main(void* _param)
{   
    log_msg_t msg;

    rt_memset(&msg, 0, sizeof(msg));

    feed_watchdog();
    mkdir(LOG_DIR, 0x777);

#ifdef ULOG_TIME_USING_TIMESTAMP
    /*check if it`s a new day*/
    static time_t now = 0;
    static struct tm *tm, tm_tmp;
    now = time(NULL);
    tm = gmtime_r(&now, &tm_tmp);
    rt_sprintf(g_today, "%02d-%02d-%04d-%02d", tm->tm_mday, tm->tm_mon + 1, tm->tm_year+1900, 0);

#else
    rt_sprintf(g_today, "%02d-%02d-%04d-%02d", 01, 01, 1970, 00);
#endif
    get_files_info();
    while(1)
    {
        log_async_waiting_log(RT_WAITING_FOREVER);
        log_async_output();
    }
}

#ifndef RT_USING_HEAP
static struct rt_thread log_thread;
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t log_thread_stack[TASK_STACK_SIZE_LOG];
#endif

int log_init(void)
{
    if (log_obj.isinit != 0) return 0;
    
    if (spi_flash_check_initial() < E_SPI_FLASH_FS_MOUNT_OK ) return -1;
    
    log_obj.async_rbb = rt_rbb_create(RT_ALIGN(LOG_ASYNC_OUTPUT_BUF_SIZE, RT_ALIGN_SIZE), LOG_ASYNC_OUTPUT_STORE_LINES);
    if (log_obj.async_rbb == RT_NULL)
    {
        SDEBUG("Error: log init failed! No memory for async rbb.\n");
        return -1;
    }

#ifdef RT_USING_HEAP
    rt_thread_t thread = rt_thread_create("log", task_main, RT_NULL, TASK_STACK_SIZE_LOG, TASK_PRIORITY_LOG, 20);
    if (thread != RT_NULL) {
        rt_thread_startup(thread);
        log_obj.isinit = 1;
    } else {
        rt_rbb_destroy(log_obj.async_rbb);
        return -1;
    }

#else
    rt_err_t result;
    result = rt_thread_init(&log_thread,
                           "log",
                           log_task_main,
                           RT_NULL,
                           &log_thread_stack[0],
                           sizeof(log_thread_stack),
                           TASK_PRIORITY_LOG,
                           20);

	if (result == RT_EOK) {
		rt_thread_startup(&log_thread);
		log_obj.isinit = 1;
	}			   
	else {
        rt_rbb_destroy(log_obj.async_rbb);
		return -1;
	}
#endif
    
    rt_sem_init(&log_obj.async_notice, "log", 0, RT_IPC_FLAG_FIFO);
    return 0;
}
INIT_APP_EXPORT(log_init);

uint8_t log_is_init(void)
{
    return log_obj.isinit;
}




//------------------------------------------------------------------------------------------------------------------

//#include "finsh.h"

//static void log_cleanup(uint8_t argc, char **argv)
//{
//    //check_bfree();
//    //clean_up();
//}
//MSH_CMD_EXPORT(log_cleanup, check log system free blocks.);








//------------------------------------------------------------------------------------------------------------------
#if 0

/*
static int check_bfree(void)
{
    int result = 0;
    struct statfs buffer;
    
    SDEBUG("check free......\n");
    feed_watchdog();
    result = dfs_statfs("/", &buffer); 
    if (result != 0)
    {
        SDEBUG("dfs_statfs failed.\n");
        return -1;  
    }
    
    if(buffer.f_bfree <= 2)
    {//less than 2 free blocks, clean up
        clean_up();
    }
    else {
        SDEBUG("free blocks : %d \n", buffer.f_bfree);
    }
    return 0;
}
*/


#define NO_RTC_DATETIME 0
#define DF_LOG_FILE_MAX 32 //31 normalday + 1 fooday
static uint32_t file_array[DF_LOG_FILE_MAX] = {0};
static int clean_up(void)
{
    int file_cur = 0;
    int file_cnt = 0;
    DIR *dir = RT_NULL;
    static char fullpath[32] = {0};

    rt_memset(file_array, 0, sizeof(file_array));
    feed_watchdog();
    dir = opendir(LOG_DIR);
    if (dir != RT_NULL)
    {
        struct dirent* dirent;
        struct stat s;
        do 
        {
            feed_watchdog();
            dirent = readdir(dir);
            if (dirent == RT_NULL) {
                break;
            }
            
            rt_memset(&s, 0, sizeof(struct stat));
            // build full path for each file 
            rt_memset(fullpath, 0, sizeof(fullpath));
            rt_sprintf(fullpath, "%s/%s", LOG_DIR, dirent->d_name);
            stat(fullpath, &s);
            if ( s.st_mode & S_IFDIR ) {
                ;
            }
            else {
                file_cur = exchange_filename(dirent->d_name);
                if (file_cur == NO_RTC_DATETIME) continue;
                
                if (file_cnt < DF_LOG_FILE_MAX) 
                {
                    file_array[file_cnt] = file_cur;
                    //sort min-->max
                    if (file_cnt >= 1) 
                    {
                        if (file_array[file_cnt] < file_array[file_cnt-1])
                        {
                            int temp = file_array[file_cnt-1];
                            file_array[file_cnt-1] = file_array[file_cnt];
                            file_array[file_cnt] = temp;
                        }
                    }
                    file_cnt ++;
                }
                else {
                    file_cnt ++;
                }
            }
        } while (dirent != RT_NULL);
        
        feed_watchdog();
        closedir(dir);
    }
    else {
        return -1;
    }

    /*check normal file*/
    if (file_cnt > DF_LOG_FILE_MAX) 
    {
        int delet_cnt = file_cnt - DF_LOG_FILE_MAX;
        if (delet_cnt > DF_LOG_FILE_MAX) delet_cnt = DF_LOG_FILE_MAX;
        for (int i = 0; i < delet_cnt; i++) 
        {
            file_cur = file_array[i];
            rt_memset(fullpath, 0, sizeof(fullpath));
            rt_sprintf(fullpath, "%s/%02d-%02d-%04d", LOG_DIR, file_cur%10000/100, file_cur%10000%100, file_cur/10000);
            feed_watchdog();
            if (unlink(fullpath) == 0) {
                SDEBUG("delete %s suceess.\n", fullpath);
            } else {
                SDEBUG("delete %s fail.\n", fullpath);
            }
        }
    }

    /*check NO_RTC_DATETIME file*/
    {
        struct stat buf;
        file_cur = NO_RTC_DATETIME;
        rt_memset(fullpath, 0, sizeof(fullpath));
        rt_sprintf(fullpath, "%s/%02d-%02d-%04d", LOG_DIR, file_cur%10000/100, file_cur%10000%100, file_cur/10000);
        if (stat(fullpath, &buf) == 0) {
            if (buf.st_size >= 500*1024) {//500KB
                if (unlink(fullpath) == 0) {
                    SDEBUG("delete %s suceess.\n", fullpath);
                } else {
                    SDEBUG("delete %s fail.\n", fullpath);
                }
            }
        }
    }
    return 0;
}
#endif






