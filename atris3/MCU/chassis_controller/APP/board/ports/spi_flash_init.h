#ifndef __SPI_FLASH_INIT_H__
#define __SPI_FLASH_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif
    
enum
{
    E_SPI_FLASH_ERR = 0,
    E_SPI_FLASH_PROBE_OK,
    E_SPI_FLASH_FAL_OK,
    E_SPI_FLASH_EF_OK,
    E_SPI_FLASH_FS_MOUNT_OK,
};
    
uint8_t spi_flash_check_initial(void);
rt_err_t spi_flash_init(void);
    
#ifdef __cplusplus
}
#endif

#endif



