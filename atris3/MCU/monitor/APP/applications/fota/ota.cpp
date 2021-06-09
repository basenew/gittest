/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : ota.cpp
 * Brief      : ota process with tinyros

 * Change Logs
 * Date           Author          Notes
 * 2020-08-05     wuxiaofeng      first version        
 */
#include "tiny_ros/ros/string.h"
#include <rtthread.h>
#include <fal.h>
#include "app_cfg.h"
#include "msg_canpkg.h"
#include "ota.h"
#include "rt_fota.h"
#include "easyflash.h"
#include "webclient.h"
#include "md5.h"
#include "spi_flash_init.h"

#define LOG_TAG              "ota"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

#define DEBUG_RT_KPRINTF (1)


const char* ConfigUrl = "monitor/config.yaml";
const char* AppUrl    = "monitor/app.rbl";
const char* BlUrl     = "monitor/bootloader.rbl";

enum
{
    SRC_PART_DOWNLOAD = 0,
    SRC_PART_BACKUP,
};

enum
{
    TAR_PART_APP = 0,
    TAR_PART_BL,
};

#define RECV_BUFSZ 128
typedef struct
{
    uint16_t url_len;
    char url[RECV_BUFSZ];
    uint32_t fw_size;
    uint8_t errcode;
    uint8_t percent;
    uint8_t forceup;
    uint8_t source_part;
    uint8_t target_part;
    char c_ver[24];
    char t_ver[24];
    uint8_t md5[32];
} ota_info_t;

static uint8_t DataBuffer[RECV_BUFSZ] = {0};
static ota_info_t OtaInfo = {0};
static struct rt_event Msg_Event;
#define EVENT_START (1 << 0)
#define EVENT_CHECK (1 << 1)

#define GLOBAL_BUFFER_SIZE (4096)
static uint8_t gBuffer[GLOBAL_BUFFER_SIZE];

static int32_t post_ota_process(void);
static int32_t configfile_download(void);
static int32_t fw_download(void);
static int32_t fw_check_md5(uint8_t src);
static int32_t enter_upgrade(uint8_t src, uint8_t tar);
static void report_when_connected(void);
static int32_t upgrade_process(void);
static int32_t wait_for_events(void);


static int32_t msg_0001_callback(const tinyros::atris_msgs::CanPkg &msg)
{
    OtaInfo.errcode = RT_FOTA_INITIAL_VAL_ERR;
    OtaInfo.url_len = msg.data_s.size();
    if (OtaInfo.url_len == 0 || OtaInfo.url_len > sizeof(OtaInfo.url)) {
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_URL_LEN_ERR;
    } 
    else {
        rt_memset(OtaInfo.url, 0, sizeof(OtaInfo.url));
        strcpy(OtaInfo.url, msg.data_s.c_str());
    }
    rt_event_send(&Msg_Event, EVENT_START);
    return 0;
}

static int32_t msg_0003_callback(const tinyros::atris_msgs::CanPkg &msg)
{
    rt_event_send(&Msg_Event, EVENT_CHECK);
    return 0;
}

static void task_main(void* param)
{
    int32_t network_status = -1;
    
    rt_event_init(&Msg_Event, "ota_evt", RT_IPC_FLAG_FIFO);
    canpkg_subscribe_cb_add(0x0001, msg_0001_callback);
    canpkg_subscribe_cb_add(0x0003, msg_0003_callback);
   
    while(1)
    {
        if (canpkg_pulisher_negotiated_get() < 0 || tinyros_nh_ok_get() < 0) 
        {/*network has some problems*/
            if (network_status == 0) {
                network_status = -1;
                LOG_I("Socket DisConnected.");
            }
            rt_thread_mdelay(1000); 
        }
        else 
        {
            if (network_status == -1) {
                LOG_I("Socket Connected.");
                network_status = 0;
                report_when_connected();
            }
            while(1)
            {
                if (wait_for_events() < 0) {
                    /*wait for events timeout*/
                    break;
                }
            }
        }
    }
}

static int32_t post_ota_process(void)
{
    tinyros::atris_msgs::CanPkg msg;
    msg.cmd = 0x0002;
    msg.data_i[0] = OtaInfo.errcode;
    msg.data_i[1] = OtaInfo.percent;
    
    rt_memset(DataBuffer, 0, sizeof(DataBuffer));
    rt_sprintf((char*)DataBuffer, "%s&%s", OtaInfo.c_ver, OtaInfo.t_ver);
    msg.data_s = (char*)DataBuffer;
    
    LOG_D("post: cmd=%04d errcode=%d percent=%d data_s=%s", msg.cmd, msg.data_i[0], msg.data_i[1], DataBuffer);

    return canpkg_publish_msg(msg);
}

#define GET_HEADER_BUFSZ 1024
static int32_t configfile_download(void)
{
    #define READ_BUFF_SIZE GLOBAL_BUFFER_SIZE  
    int ret = -RT_ERROR, resp_status = -1;
    int file_size = 0, length, total_length = 0;
    rt_uint8_t *buffer_read = RT_NULL;
    struct webclient_session* session = RT_NULL;
    char* ptrh = RT_NULL;
    char* ptrt = RT_NULL;
    const char* ptmp = RT_NULL;
    
    LOG_I("Downloading config file...");

    /* create webclient session and set header response size */
    session = webclient_session_create(GET_HEADER_BUFSZ);
    if (!session)
    {
        LOG_E("open uri failed.");
        goto __exit;
    }

    /* get full url*/
    rt_memset(DataBuffer, 0, sizeof(DataBuffer));
    rt_sprintf((char*)&DataBuffer[0], "%s/%s", OtaInfo.url, ConfigUrl);
    
    LOG_I("Config file Url: %s", DataBuffer);
    
    /* send GET request by default header */
    if ((resp_status = webclient_get(session, (const char*)DataBuffer)) != 200)
    {
        LOG_E("webclient GET request failed, response(%d) error.", resp_status);
        goto __exit;
    }

    file_size = webclient_content_length_get(session);
    LOG_I("config file_size:%d\n",file_size);

    if (file_size == 0)
    {
        LOG_E("Request file size is 0!");
        goto __exit;
    }
    else if (file_size < 0)
    {
        LOG_E("webclient GET request type is chunked.");
        goto __exit;
    }
    
    //buffer_read = (rt_uint8_t *)web_malloc(READ_BUFF_SIZE);
    buffer_read = gBuffer;
    if (buffer_read == RT_NULL)
    {
        LOG_E("No memory for http ota!");
        goto __exit;
    }
    rt_memset(buffer_read, 0x00, READ_BUFF_SIZE);
    
    do
    {
        length = webclient_read(session, buffer_read, file_size - total_length > READ_BUFF_SIZE ?
                            READ_BUFF_SIZE : file_size - total_length);   
        if (length > 0)
        {
            total_length += length;
        }
        else
        {
            LOG_E("Exit: server return err (%d)!", length);
            goto __exit;
        }
    } while(total_length != file_size);

    if (total_length == file_size)
    {
        LOG_I("Download config files success.");
    }
    else {
        goto __exit;
    }
    
    /* GET target_part*/
    ptrh = RT_NULL;
    ptrt = RT_NULL;
    rt_memset(DataBuffer, 0, sizeof(DataBuffer));
    ptrh = rt_strstr((const char*)buffer_read, "target_part:");
    if (ptrh) {
        ptrt = rt_strstr((const char*)ptrh, "\n");
        if (ptrt == RT_NULL) {
            goto __exit;
        }
        ptrh += rt_strlen("target_part:");
        rt_memcpy(DataBuffer, ptrh, ptrt - ptrh);
        LOG_I("[config]target_part: %s\n", DataBuffer);
    }
    else {
        goto __exit;
    }
    
    if (!rt_strcmp((const char*)DataBuffer, RT_FOTA_APP_PART_NAME)) {
        OtaInfo.target_part = TAR_PART_APP;
    }
    else if(!rt_strcmp((const char*)DataBuffer, RT_FOTA_BL_PART_NAME)) {
        OtaInfo.target_part = TAR_PART_BL;
    } 
    else {
        goto __exit;
    }
    
    /* GET source_part*/
    ptrh = RT_NULL;
    ptrt = RT_NULL;
    rt_memset(DataBuffer, 0, sizeof(DataBuffer));
    ptrh = rt_strstr((const char*)buffer_read, "source_part:");
    if (ptrh) {
        ptrt = rt_strstr((const char*)ptrh, "\n");
        if (ptrt == RT_NULL) {
            goto __exit;
        }
        ptrh += rt_strlen("source_part:");
        rt_memcpy(DataBuffer, ptrh, ptrt - ptrh);
        LOG_I("[config]source_part: %s\n", DataBuffer);
    }
    else {
        goto __exit;
    }
    if (!rt_strcmp((const char*)DataBuffer, RT_FOTA_DL_PART_NAME)) {
        OtaInfo.source_part = SRC_PART_DOWNLOAD;
    }
    else if(!rt_strcmp((const char*)DataBuffer, RT_FOTA_BK_PART_NAME)) {
        OtaInfo.source_part = SRC_PART_BACKUP;
    } 
    else {
        goto __exit;
    }

    /* GET software_version*/
    ptrh = RT_NULL;
    ptrt = RT_NULL;
    ptmp = RT_NULL;
    if (OtaInfo.target_part == TAR_PART_APP) {
        ptmp =  "app_version:";
    } else {
        ptmp =  "bl_version:";
    }
    ptrh = rt_strstr((const char*)buffer_read, ptmp);
    if (ptrh) {
        ptrt = rt_strstr((const char*)ptrh, "\n");
        if (ptrt == RT_NULL) { 
            goto __exit;
        }
        ptrh += rt_strlen(ptmp);
        if ((ptrt - ptrh) <= sizeof(OtaInfo.t_ver)) {
            rt_memset(OtaInfo.t_ver, 0, sizeof(OtaInfo.t_ver));
            rt_memcpy(OtaInfo.t_ver, ptrh, ptrt - ptrh);
            LOG_I("[config]t_ver: %s\n", OtaInfo.t_ver);
        }
        else {
            goto __exit;
        }
    }
    else {
        goto __exit;
    }

    /* GET forceUp*/
    ptrh = RT_NULL;
    ptrt = RT_NULL;
    rt_memset(DataBuffer, 0, sizeof(DataBuffer));
    ptrh = rt_strstr((const char*)buffer_read, "forceUp:");
    if (ptrh) {
        ptrt = rt_strstr((const char*)ptrh, "\n");
        if (ptrt == RT_NULL) { 
            goto __exit;
        }
        ptrh += rt_strlen("forceUp:");
        rt_memcpy(DataBuffer, ptrh, ptrt - ptrh);
        LOG_I("[config]forceUp: %s\n", DataBuffer);
    }
    else {
        goto __exit;
    }
    if (!rt_strcmp((const char*)DataBuffer, "false")) {
        OtaInfo.forceup = 0;
    }
    else {
        OtaInfo.forceup = 1;
    }
    
    /* GET MD5*/
    ptrh = RT_NULL;
    ptrt = RT_NULL;
    ptmp = RT_NULL;
    if (OtaInfo.target_part == TAR_PART_APP) {
        ptmp =  "app_md5:";
    } else {
        ptmp =  "bl_md5:";
    }
    ptrh = rt_strstr((const char*)buffer_read, ptmp);
    if (ptrh) {
        ptrt = rt_strstr((const char*)ptrh, "\n");
        if (ptrt == RT_NULL) { 
            goto __exit;
        }
        ptrh += rt_strlen(ptmp);
        if ((ptrt - ptrh) == sizeof(OtaInfo.md5)) {
            rt_memset(OtaInfo.md5, 0, sizeof(OtaInfo.md5));
            rt_memcpy(OtaInfo.md5, ptrh, ptrt - ptrh);
            LOG_I("[config]MD5: %s\n", OtaInfo.md5);
        } else {
            goto __exit;
        }
    }
    else {
        goto __exit;
    }
    
    ret = RT_EOK;
    
__exit:
    if (ret != RT_EOK) {
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_CONFIGFILE_ERR;
    }
    rt_memset(DataBuffer, 0, sizeof(DataBuffer));
    if (session != RT_NULL)
        webclient_close(session);
    /*
    if (buffer_read != RT_NULL)
        web_free(buffer_read);
    */
    rt_memset(buffer_read, 0, READ_BUFF_SIZE);

    return ret;
}

static int32_t print_progress(size_t cur_size, size_t total_size)
{
    int percnt = cur_size * 100 / total_size;
    if (percnt > 100) percnt = 100;
    
    rt_kprintf("Downloading: %d%%\033[1A\r\n", percnt); 
    OtaInfo.percent = percnt;
    return post_ota_process();
}

static int32_t fw_download(void)
{
    #define HTTP_OTA_BUFF_LEN  GLOBAL_BUFFER_SIZE
    int32_t ret = -RT_ERROR, resp_status = -1;
    int32_t file_size = 0, length, total_length = 0;
    rt_uint8_t *buffer_read = RT_NULL;
    struct webclient_session* session = RT_NULL;
    const struct fal_partition * dl_part = RT_NULL;
    uint32_t tarpart_size_max = 0;
    const char* tar_partname = RT_NULL;
    
    if (OtaInfo.source_part == SRC_PART_DOWNLOAD) {
        tar_partname = RT_FOTA_DL_PART_NAME;
    }
    else {
        tar_partname = RT_FOTA_BK_PART_NAME;
    }
    
    LOG_I("Downloading fireware...");
    
    /* create webclient session and set header response size */
    session = webclient_session_create(GET_HEADER_BUFSZ);
    if (!session)
    {
        LOG_E("open uri failed.");
        goto __exit;
    }
    
    /* get full url*/
    rt_memset(DataBuffer, 0, sizeof(DataBuffer));
    if (OtaInfo.target_part == TAR_PART_APP) {
        rt_sprintf((char*)&DataBuffer[0], "%s/%s", OtaInfo.url, AppUrl);
    }
    else {
        rt_sprintf((char*)&DataBuffer[0], "%s/%s", OtaInfo.url, BlUrl);
    }
    
    LOG_I("Fireware Url: %s", DataBuffer);
    
    /* send GET request by default header */
    if ((resp_status = webclient_get(session, (const char*)DataBuffer)) != 200)
    {
        LOG_E("webclient GET request failed, response(%d) error.", resp_status);
        goto __exit;
    }

    file_size = webclient_content_length_get(session);
    LOG_I("Fireware file_size:%d\n",file_size);

    if (OtaInfo.target_part == TAR_PART_APP) {
        tarpart_size_max = APP_SIZE;
    } else {
        tarpart_size_max = BOOTLOADER_SIZE;
    }
    
    if (file_size <= 0 || file_size > tarpart_size_max) {
        LOG_E("Request file size is out of range!");
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_SIZE_ERR;
        goto __exit;
    }

    /* Get download partition information and erase download partition data */
    if ((dl_part = fal_partition_find(tar_partname)) == RT_NULL)
    {
        LOG_E("Firmware download failed! Partition (%s) find error!", tar_partname);
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_FINDPART_ERR;
        goto __exit;
    }

    LOG_I("Start erase flash (%s) partition!", dl_part->name);

    if (fal_partition_erase(dl_part, 0, file_size) < 0)
    {
        LOG_E("Firmware download failed! Partition (%s) erase error!", dl_part->name);
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_FINDPART_ERR;
        goto __exit;
    }
    LOG_I("Erase flash (%s) partition success!", dl_part->name);

    //buffer_read = web_malloc(HTTP_OTA_BUFF_LEN);
    buffer_read = gBuffer;
    if (buffer_read == RT_NULL)
    {
        LOG_E("No memory for http ota!");
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_MEM_ERR;
        goto __exit;
    }
    rt_memset(buffer_read, 0x00, HTTP_OTA_BUFF_LEN);
    
    LOG_I("Fireware file size is (%d)", file_size);

    LOG_I("Start download firmware to Partition(%s)", dl_part->name);
    
    do
    {
        length = webclient_read(session, buffer_read, file_size - total_length > HTTP_OTA_BUFF_LEN ?
                            HTTP_OTA_BUFF_LEN : file_size - total_length);   
        if (length > 0)
        {
            if (fal_partition_write(dl_part, total_length, buffer_read, length) < 0)
            {
                LOG_E("Firmware download failed! Partition (%s) write data error!", dl_part->name);
                OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_FLASH_WRITE_ERR;
                goto __exit;
            }
            total_length += length;
            print_progress(total_length, file_size);
        }
        else
        {
            LOG_E("Exit: server return err (%d)!", length);
            OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_NETWORK_ERR;
            goto __exit;
        }

    } while(total_length != file_size);
   
    if (total_length == file_size) {
        OtaInfo.fw_size = file_size;
        LOG_I("Download firmware to Partition(%s) success.", dl_part->name);
    }
    else {
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_INCOMPLETE_ERR;
        goto __exit;
    }

    ret = RT_EOK;
__exit:
    
    rt_memset(DataBuffer, 0, sizeof(DataBuffer));
    if (session != RT_NULL)
        webclient_close(session);
    /*
    if (buffer_read != RT_NULL)
        web_free(buffer_read);
    */
    rt_memset(buffer_read, 0, HTTP_OTA_BUFF_LEN);

    return ret;
}

static int32_t fw_check_md5(uint8_t src)
{
    int32_t ret = -RT_ERROR;
    const struct fal_partition *part;
    rt_uint8_t *body_buf = RT_NULL;
    int32_t body_read_len = 0;
    char md5_str[32] = {0};
    
    const char* tar_partname = RT_NULL;
    if (src == SRC_PART_DOWNLOAD) {
        tar_partname = RT_FOTA_DL_PART_NAME;
    }
    else {
        tar_partname = RT_FOTA_BK_PART_NAME;
    }
    part = fal_partition_find(tar_partname);
    
    MD5Init(&gMd5 );    
    
    body_buf = (rt_uint8_t *)rt_malloc(64);
	for (int body_pos = 0; body_pos < OtaInfo.fw_size;)
	{	
		body_read_len = fal_partition_read(part, body_pos, body_buf, 64);      
		if (body_read_len > 0) 
		{
            if ((body_pos + body_read_len) > OtaInfo.fw_size)
            {
                body_read_len = OtaInfo.fw_size - body_pos;
            }
            MD5Update(&gMd5, body_buf, body_read_len);
			body_pos = body_pos + body_read_len;
		}
		else
		{
			LOG_I("Partition[%s] read error while check md5!", part->name);		
			goto __exit;
		}
	}
    rt_memset(DataBuffer, 0, sizeof(DataBuffer));
    MD5Final(DataBuffer, &gMd5);
    
#if DEBUG_RT_KPRINTF    
    rt_kprintf("fireware md5: \r\n");
    for(uint8_t i = 0; i < 16; i++) {
        rt_kprintf("%02X", DataBuffer[i]);
    }
    rt_kprintf("\r\n");
#endif
    
    ConvertUnCharToStr(md5_str, DataBuffer, 16);
    if (rt_strcmp((const char*)&md5_str[0], (const char*)&OtaInfo.md5[0])) {
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_MD5_ERR;
        LOG_E("Exit: MD5 error!");
        goto __exit;
    }
    
    ret = RT_EOK;
    
__exit:
    if (body_buf) 
        rt_free(body_buf);
    return ret;
}

static int32_t enter_upgrade(uint8_t src, uint8_t tar) 
{
    static char* fota_cmd[1][4] = {
        {0, 0, 0, 0},
    }; 

    static const char* name = "fota";
    static const char* param = "upgrade";
    static const char* src_dl = RT_FOTA_DL_PART_NAME;
    static const char* src_bk = RT_FOTA_BK_PART_NAME;
    static const char* tar_app = RT_FOTA_APP_PART_NAME;
    static const char* tar_bl = RT_FOTA_BL_PART_NAME;

    fota_cmd[0][0] = (char*)name;
    fota_cmd[0][1] = (char*)param;
    
    if (src == SRC_PART_DOWNLOAD) {
        fota_cmd[0][2] = (char*)src_dl;        
    } else {
        fota_cmd[0][2] = (char*)src_bk; 
    }
    
    if (tar == TAR_PART_APP) {
        fota_cmd[0][3] = (char*)tar_app;        
    } else {
        fota_cmd[0][3] = (char*)tar_bl; 
    }
    
    if (OtaInfo.forceup != 0) {
        fota_flags_t* pflags = fota_flags_get();
        pflags->uflag.sflag.force_up = FLAG_YES;
    } 
    LOG_I("%s %s %s %s \n", fota_cmd[0][0], fota_cmd[0][1], fota_cmd[0][2], fota_cmd[0][3]);
    rt_fota(4, fota_cmd[0]);
    
    return 0;
}

static void report_when_connected(void)
{
    /*target ver*/
    static char value[24 + 1];
    int32_t get_size = 0;
    char* cvalue = RT_NULL;
    
    fota_flags_t* pflags = fota_flags_get();
    
    OtaInfo.percent = 0;
    
    /*current ver*/
    rt_sprintf(OtaInfo.c_ver, "%s", pflags->app_version);
    /*errcode*/
    OtaInfo.errcode = pflags->errcode;
    if (OtaInfo.errcode == RT_FOTA_HEADER_SW_CHECK_ERR) {
        OtaInfo.errcode = RT_FOTA_NO_ERR;
    }
    /*report ota result if ota_upgrade_before_reset */
    cvalue = ef_get_env("ota_upgrade_before_reset");
    if (!rt_strcmp(cvalue, "bl")) {
        ef_set_and_save_env("ota_upgrade_before_reset", "no");
        /*target ver*/
        get_size = ef_get_env_blob("ota_t_ver_bl", value, sizeof(value), NULL);
        rt_memcpy(OtaInfo.t_ver, value, get_size);
        post_ota_process();
    }
    else if (!rt_strcmp(cvalue, "app")) {
        ef_set_and_save_env("ota_upgrade_before_reset", "no");
        /*target ver*/
        get_size = ef_get_env_blob("ota_t_ver_app", value, sizeof(value), NULL);
        rt_memcpy(OtaInfo.t_ver, value, get_size);
        post_ota_process();
    }
    else {
        rt_sprintf(OtaInfo.t_ver, "%s", "0.0.0");;
    }
    /*recover the t_ver to app version for default*/
    get_size = ef_get_env_blob("ota_t_ver_app", value, sizeof(value), NULL);
    rt_memcpy(OtaInfo.t_ver, value, get_size);
}

static int32_t upgrade_process(void)
{
    int32_t ret = -RT_ERROR;
    
    do
    {
        if (OtaInfo.errcode != RT_FOTA_INITIAL_VAL_ERR) {
            LOG_E("errcode: %d", OtaInfo.errcode);
            post_ota_process();
            break;
        }
        
        if (configfile_download() != RT_EOK) {
            LOG_E("errcode: %d", OtaInfo.errcode);
            post_ota_process();
            break;
        }
        
        if (fw_download() != RT_EOK) {
            LOG_E("errcode: %d", OtaInfo.errcode);
            post_ota_process();
            break;
        }
            
        if (fw_check_md5(OtaInfo.source_part) != RT_EOK) {
            LOG_E("errcode: %d", OtaInfo.errcode);
            post_ota_process();
            break;
        }

        if (OtaInfo.target_part == TAR_PART_APP) {
            ef_set_and_save_env("ota_upgrade_before_reset", "app");
            ef_set_and_save_env("ota_t_ver_app", OtaInfo.t_ver);
        } else {
            ef_set_and_save_env("ota_upgrade_before_reset", "bl");
            ef_set_and_save_env("ota_t_ver_bl", OtaInfo.t_ver);
        }

        enter_upgrade(OtaInfo.source_part, OtaInfo.target_part);
        ret = RT_EOK;
    } while(0);
    return ret;
}

static int32_t wait_for_events(void)
{
    int32_t ret = -RT_ERROR;
    rt_uint32_t e;

    if (rt_event_recv(&Msg_Event, (EVENT_START | EVENT_CHECK), RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, \
        rt_tick_from_millisecond(1000), &e) == RT_EOK)
    {
        if (e & EVENT_START)
        {
            upgrade_process();
        }
        
        if(e & EVENT_CHECK)
        {
            post_ota_process();
        }
        ret = RT_EOK;
    }
    return ret;
}

int32_t ota_init(void)
{
	if (spi_flash_check_initial() < E_SPI_FLASH_EF_OK) return -1;
	
    rt_thread_t thread = rt_thread_create("ota", task_main, RT_NULL, TASK_STACK_SIZE_OTA, TASK_PRIORITY_OTA, 20);
    if (thread != RT_NULL) {
        rt_thread_startup(thread);
    }
    else{
        return -1;
    }
    return 0;
}



int ota_test(uint8_t argc, char **argv) {
    tinyros::atris_msgs::CanPkg msg;
    msg.cmd = 0x0001;

    msg.data_s = "http://10.20.18.2/hfs/";
    canpkg_publish_msg(msg);
    return 0;
}
MSH_CMD_EXPORT(ota_test, ota test);







/*
static int32_t msg_callback(const tinyros::atris_msgs::CanPkg &msg)
{
    static char* cmd[1][4] = {
        {0, 0, 0, 0},
    }; 

    static const char* cmd_name = "http_ota";
    static const char* src_dl = "download";
    static const char* src_bk = "backup";
    static const char* tar_app = "app";
    static const char* tar_bl = "bl";

    cmd[0][0] = (char*)cmd_name;

    static char c_url[64] = {0};
    rt_memset(c_url, 0, sizeof(c_url));
    strcpy(c_url, msg.data_s.c_str());
    cmd[0][1] = &c_url[0];

    if (msg.data_c[0] == PRO_SRC_DOWNLOAD) {
        cmd[0][2] = (char*)src_dl;
    } else if (msg.data_c[0] == PRO_SRC_BACKUP) {
        cmd[0][2] = (char*)src_bk;
    } else {
        return -1;
    }

    if (msg.data_c[1] == PRO_SRC_DOWNLOAD) {
        cmd[0][3] = (char*)tar_app;
    } else if (msg.data_c[1] == PRO_SRC_BACKUP) {
        cmd[0][3] = (char*)tar_bl;
    } else {
        return -1;;
    }
    rt_kprintf("%s %s %s %s \n", cmd[0][0], cmd[0][1], cmd[0][2], cmd[0][3]);
    http_ota(4, cmd[0]);
    return 0;
}
*/







