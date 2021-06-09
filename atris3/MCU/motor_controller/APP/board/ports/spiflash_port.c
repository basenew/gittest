#include <rtthread.h>
#include "dfs_fs.h"
#include "fal.h"
#include <dfs_posix.h>


#if defined(BSP_USING_SPI_FLASH)

#define LOG_TAG              "spiflash"
#define LOG_LVL              LOG_LVL_INFO
#include <rtdbg.h>


#define FS_PARTITION_NAME "fs"

static int stm32_spiflash_mount(void)
{
    struct rt_device *mtd_dev = RT_NULL;
    
    fal_init();
    
    mtd_dev = fal_mtd_nor_device_create(FS_PARTITION_NAME);
    if (!mtd_dev)
    {
        LOG_W("Can't create a mtd device on '%s' partition.\n", FS_PARTITION_NAME);
    }
    else
    {
        if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
        {
            LOG_D("mount '%s' partition sucessful.\n", FS_PARTITION_NAME);
            
#if defined(BSP_USING_SDCARD)
            //mount sdcard to /sdcard
            mkdir("/sdcard", 0x777);
#endif
        }
        else
        {
            dfs_mkfs("lfs", FS_PARTITION_NAME);
            //NOTE: 擦除flash后，第一次重启要先格式化flash，再mount
            if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
            {
                rt_kprintf("mount '%s' partition sucessful.\n", FS_PARTITION_NAME);
            }
            else {
                rt_kprintf("mount '%s' partition failed.\n", FS_PARTITION_NAME);
            }
            //rt_hw_cpu_reset(); 如果有问题会导致一直重启
        }
    }

    return 0;
}
INIT_ENV_EXPORT(stm32_spiflash_mount);

//#include "spi_flash_sfud.h"
//static void fs_remkfs(uint8_t argc, char **argv)
//{
//    sfud_err result = SFUD_SUCCESS;
//    rt_spi_flash_device_t rtt_dev = NULL;
//    const sfud_flash *sfud_dev = NULL;
//    
//    if ((argc >= 2 && rt_strcmp(argv[1], "yes")) || argc < 2) {
//        rt_kprintf("DANGER: It will erase full chip, and then reset the mcu core! Please run 'fs_remkfs yes'.\n");
//        return;
//    }
//    
//    
//    rtt_dev = rt_sfud_flash_probe(FAL_USING_NOR_FLASH_DEV_NAME, "spi20");
//    
//    sfud_dev = (sfud_flash_t)rtt_dev->user_data;
//    
//    result = sfud_erase(sfud_dev, 0, NOR_FLASH_DEV_CAPACITY);
//    if (result != SFUD_SUCCESS) {
//        rt_kprintf("Erase the %s full flashchip fail.\n");
//        return;
//    }
//    
//    dfs_mkfs("lfs", FS_PARTITION_NAME);
//    
//    rt_hw_cpu_reset();
//    
//}
//MSH_CMD_EXPORT(fs_remkfs, remkfs the flash.);


#endif /* defined(BSP_USING_SPI_FLASH) */

