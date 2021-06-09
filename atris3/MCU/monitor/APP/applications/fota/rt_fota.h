#ifndef _RT_FOTA_H_
#define _RT_FOTA_H_

#ifdef __CC_ARM                         /* ARM Compiler */
    #define RT_FOTA_WEAK                __weak
#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
    #define RT_FOTA_WEAK                __weak
#elif defined (__GNUC__)                /* GNU GCC Compiler */
    #define RT_FOTA_WEAK                __attribute__((weak))
#endif /* __CC_ARM */


#ifdef __cplusplus
extern "C" {
#endif
    
#include "rt_fota_cfg.h"
#include "rt_fota_flag.h"

/**
 * FOTA firmware encryption algorithm and compression algorithm
 */
enum rt_fota_algo
{
    RT_FOTA_CRYPT_ALGO_NONE    = 0x0L,               /**< no encryption algorithm and no compression algorithm */
    RT_FOTA_CRYPT_ALGO_XOR     = 0x1L,               /**< XOR encryption */
    RT_FOTA_CRYPT_ALGO_AES256  = 0x2L,               /**< AES256 encryption */
    RT_FOTA_CMPRS_ALGO_GZIP    = 0x1L << 8,          /**< Gzip: zh.wikipedia.org/wiki/Gzip */
    RT_FOTA_CMPRS_ALGO_QUICKLZ = 0x2L << 8,          /**< QuickLZ: www.quicklz.com */
    RT_FOTA_CMPRS_ALGO_FASTLZ  = 0x3L << 8,          /**< FastLZ: fastlz.org/ */

    RT_FOTA_CRYPT_STAT_MASK    = 0xFL,
    RT_FOTA_CMPRS_STAT_MASK    = 0xFL << 8,
};
typedef enum rt_fota_algo rt_fota_algo_t;


/* FOTA error code */
typedef enum {
    RT_FOTA_NO_ERR             =  0,

    /*header errors*/
    RT_FOTA_HEADER_CHECK_ERR   =  1,
    RT_FOTA_HEADER_FIND_ERR    =  2,
    RT_FOTA_HEADER_READ_ERR    =  3,
    RT_FOTA_HEADER_TYPE_ERR    =  4,
    RT_FOTA_HEADER_H_CRC_ERR   =  5,
    RT_FOTA_HEADER_SUPPORT_ERR =  6,
    RT_FOTA_HEADER_NO_MEN_ERR  =  7,
    RT_FOTA_HEADER_B_CRC_ERR   =  8,
    RT_FOTA_HEADER_HW_CHECK_ERR = 9,
    RT_FOTA_HEADER_SW_CHECK_ERR = 10,

    /*fal errors*/
    RT_FOTA_FAL_CHECK_ERR      =  15,

    /*flag errors*/
    RT_FOTA_FLAG_SOURCE_ERR    =  20,

    RT_FOTA_SERIOUS_ERR_LINE   =  30,

    RT_FOTA_UPGRADE_CHECK_ERR   = 40,
    RT_FOTA_UPGRADE_FIND_ERR    = 41,
    RT_FOTA_UPGRADE_ERASE_ERR   = 42,
    RT_FOTA_UPGRADE_NO_MEM_ERR  = 43,
    RT_FOTA_UPGRADE_READ_ERR    = 44,
    RT_FOTA_UPGRADE_WRITE_ERR   = 45,
    RT_FOTA_UPGRADE_SIZECHECK_ERR = 46,
    
    RT_FOTA_DOWNLOAD_CMD_ERR = 100,
    RT_FOTA_DOWNLOAD_URL_LEN_ERR = 101,
    RT_FOTA_DOWNLOAD_CONFIGFILE_ERR = 102,
    RT_FOTA_DOWNLOAD_FW_SIZE_ERR = 108,
    RT_FOTA_DOWNLOAD_FW_FINDPART_ERR = 109,
    RT_FOTA_DOWNLOAD_FW_ERASE_ERR = 110,
    RT_FOTA_DOWNLOAD_FW_MEM_ERR = 111,
    RT_FOTA_DOWNLOAD_FW_FLASH_WRITE_ERR = 112,
    RT_FOTA_DOWNLOAD_FW_READ_ERR = 113,
    RT_FOTA_DOWNLOAD_FW_NETWORK_ERR = 114,
    RT_FOTA_DOWNLOAD_FW_INCOMPLETE_ERR = 115,
    RT_FOTA_DOWNLOAD_FW_MD5_ERR = 116,
    
    RT_FOTA_INITIAL_VAL_ERR = 255,

} rt_fota_err_t;


void rt_fota(rt_uint8_t argc, char **argv);

#ifdef __cplusplus
}
#endif

#endif /* _RT_FOTA_H_ */

