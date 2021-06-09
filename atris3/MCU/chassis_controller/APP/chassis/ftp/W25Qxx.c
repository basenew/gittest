#include <stdio.h>

#include "W25Qxx.h"
#include <fal.h>
#include "rt_fota.h"
#define LOG_TAG              "ftp"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>


//SPI_HandleTypeDef hspi1;
#define RT_FTP_BK_PART_NAME   "idle1"
const struct fal_partition * dl_part = RT_NULL;
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

static ota_info_t OtaInfo = {0};



#define DF_SECTOR_SIZE 4096
void W25Qxx_EraseSector(uint16_t sector)
{
    //return;
    rt_kprintf("%s:%d \r\n",__FUNCTION__,sector);
    if (fal_partition_erase(dl_part, sector * DF_SECTOR_SIZE, DF_SECTOR_SIZE) < 0)
    {
        LOG_E("%s (%s) erase sector(%x) error!",__FUNCTION__,sector, dl_part->name);
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_FINDPART_ERR;
        return;
    }
}

void W25Qxx_erase_all(void)
{
    /* Get download partition information and erase download partition data */
    if ((dl_part = fal_partition_find(RT_FTP_BK_PART_NAME)) == RT_NULL)
    {
        LOG_E("Firmware download failed! Partition (%s) find error!", RT_FTP_BK_PART_NAME);
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_FINDPART_ERR;
        return;
    }
    
    if (fal_partition_erase(dl_part, 0, dl_part->len) < 0)
    {
        LOG_E("%s (%s) erase sector(%x) error!",__FUNCTION__,0, dl_part->name);
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_FINDPART_ERR;
        return;
    }

    LOG_I("%s(%S) ftp file system erase OK",__FUNCTION__, dl_part->name);
    
}

void W25Qxx_Init(void)
{
    /* Get download partition information and erase download partition data */
    if ((dl_part = fal_partition_find(RT_FTP_BK_PART_NAME)) == RT_NULL)
    {
        LOG_E("Firmware download failed! Partition (%s) find error!", RT_FTP_BK_PART_NAME);
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_FINDPART_ERR;
        return;
    }

    LOG_I("%s(%S) ftp file system init OK",__FUNCTION__, dl_part->name);
    
}
void W25Qxx_ProgramPage(uint32_t addr, const void *data, int len)
{
    rt_kprintf("%s:%d %d \r\n",__FUNCTION__,addr,len);
    
    if (fal_partition_write(dl_part, addr, data, len) < 0)
    {
        LOG_E("Firmware download failed! Partition (%s) write data error!", dl_part->name);
        OtaInfo.errcode = RT_FOTA_DOWNLOAD_FW_FLASH_WRITE_ERR;
        return;
    }
}

void W25Qxx_Read(uint32_t addr, void *data, int len)
{
    int body_read_len = fal_partition_read(dl_part, addr, data, len);      
		if (body_read_len > 0) 
		{
            return;
		}
	
		LOG_I("%s read error:%x,%d",__FUNCTION__, addr,len);

}

uint8_t W25Qxx_ReadID(uint8_t *pm)
{

  return 0x14;//id;
}

