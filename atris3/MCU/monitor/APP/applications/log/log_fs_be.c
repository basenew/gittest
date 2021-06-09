/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : fs_be.c
 * Brief      : filesystem backend

 * Change Logs
 * Date           Author        Version       Notes
 * 2019-07-17     wuxiaofeng    v1.0          
 *
 */

#include <rthw.h>
#include <ulog.h>
#include "log.h"


static struct ulog_backend fs;

void ulog_fs_backend_output(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw,
                            const char *log, size_t len)
{
    if(level >= LOG_LVL_DBG) 
    {// log of LOG_LVL_DBG DONOT output to spi flash.
        return;
    }
     // check task_log is up or not.  
    if(log_is_init() != 0)
    {
        log_notify_msg(log, len);
    }
}

int ulog_fs_backend_init(void)
{
    ulog_init();
    fs.output = ulog_fs_backend_output;

    ulog_backend_register(&fs, "fs", RT_TRUE);

    return 0;
}
INIT_PREV_EXPORT(ulog_fs_backend_init);




