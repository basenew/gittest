#include <rtthread.h>
#include "common.h"
#include "app_cfg.h"
#include "msg_canpkg.h"
#include "msg_canpkg_app.h"
#include "log.h"

static int upload_start(void);

static int32_t msg_logupload_callback(const tinyros::atris_msgs::CanPkg &msg)
{
    tinyros::atris_msgs::CanPkg tx_msg;
    tx_msg.cmd = CANPKG_CMD_LOG_UPLOAD_ACK;
    if (upload_start() < 0) 
    {
        tx_msg.data_i[0] = -1;
        canpkg_publish_msg(tx_msg);
    }
    return 0;
}

static void task_main(void* _param)
{
    tinyros::atris_msgs::CanPkg tx_msg;
    tx_msg.cmd = CANPKG_CMD_LOG_UPLOAD_ACK;
    if (log_upload() > 0) {
        tx_msg.data_i[0] = 0;
    }
    else  {
        tx_msg.data_i[0] = -1;
    }
    canpkg_publish_msg(tx_msg);
}

static int upload_start(void)
{
    if (log_is_init() == 0) return -1;
if(DF_THREAD_STATIC_MEMORY == 0){    
    rt_thread_t thread = rt_thread_create("logup", task_main, RT_NULL, TASK_STACK_SIZE_LOGUPLOAD, TASK_PRIORITY_LOG, 20);
    if (thread != RT_NULL) {
        rt_thread_startup(thread);
    } else {
        return -1;
    }
  }else{
    static struct rt_thread logup_thread;
    ALIGN(RT_ALIGN_SIZE)
    static char logup_thread_stack[TASK_STACK_SIZE_LOGUPLOAD]; 
    	rt_err_t result = RT_EOK;
    result = rt_thread_init(&logup_thread,
                            "logup",
                            task_main, RT_NULL,
                            &logup_thread_stack[0], sizeof(logup_thread_stack),
                            TASK_PRIORITY_LOG, 20);

    if (result == RT_EOK)
    	rt_thread_startup(&logup_thread);
    else return -1;
    	//LOG_I("%s thread create failed.",__FUNCTION__);
    
}  
    
    return 0;  
}

int logupload_init(void)
{
    canpkg_subscribe_cb_add(CANPKG_CMD_LOG_UPLOAD_ASK, msg_logupload_callback);
    return 0;
}
INIT_APP_EXPORT(logupload_init);






