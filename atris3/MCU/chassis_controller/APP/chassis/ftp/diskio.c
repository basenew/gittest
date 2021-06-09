/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include <stdint.h>
#include <stddef.h>
#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "W25Qxx.h"

/* Definitions of physical drive number for each drive */
#define DEV_W25QXX 0

#if FF_MIN_SS != 4096 || FF_MAX_SS != 4096
#error "sector size != 4096"
#endif

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
  switch (pdrv)
  {
    case DEV_W25QXX:
      return 0; // Õý³£
  }
  return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
  switch (pdrv)
  {
    case DEV_W25QXX:
      W25Qxx_Init();
      return 0;
  }
  return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
  switch (pdrv)
  {
    case DEV_W25QXX:
      W25Qxx_Read(sector * FF_MIN_SS, buff, count * FF_MIN_SS);
      return RES_OK;
  }

  return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
  uint8_t i;
  uint32_t addr;
  
  switch (pdrv)
  {
    case DEV_W25QXX:
      addr = sector * FF_MIN_SS;
      while (count--)
      {
        W25Qxx_EraseSector(sector);
        sector++;
        
        for (i = 0; i < 16; i++)
        {
          W25Qxx_ProgramPage(addr, buff, 256);
          addr += 256;
          buff += 256;
        }
      }
      return RES_OK;
  }

  return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
  uint8_t id;
  DWORD *p = buff;
  
  switch (pdrv)
  {
    case DEV_W25QXX:
      switch (cmd)
      {
        case CTRL_SYNC:
          return RES_OK;
        case GET_SECTOR_COUNT:
          id = W25Qxx_ReadID(NULL);
          switch (id)
          {
            case 0x14: // W25Q16
              *p = 512; // 2MB / 4096 = 512
              break;
            case 0x15: // W25Q32
              *p = 1024;
              break;
            case 0x16:
              *p = 2048; // W25Q64
              break;
            case 0x17: // W25Q128
              *p = 4096; // 16MB / 4096 = 4096
              break;
            default:
              return RES_ERROR;
          }
          return RES_OK;
      }
      break;
  }

  return RES_PARERR;
}

