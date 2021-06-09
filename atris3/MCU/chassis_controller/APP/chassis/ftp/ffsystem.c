/*------------------------------------------------------------------------*/
/* Sample Code of OS Dependent Functions for FatFs                        */
/* (C)ChaN, 2018                                                          */
/*------------------------------------------------------------------------*/


#include "ff.h"
#include "W25Qxx.h"
#if FF_USE_LFN == 3	/* Dynamic memory allocation */

/*------------------------------------------------------------------------*/
/* Allocate a memory block                                                */
/*------------------------------------------------------------------------*/

void* ff_memalloc (	/* Returns pointer to the allocated memory block (null if not enough core) */
	UINT msize		/* Number of bytes to allocate */
)
{
	return malloc(msize);	/* Allocate a new memory block with POSIX API */
}


/*------------------------------------------------------------------------*/
/* Free a memory block                                                    */
/*------------------------------------------------------------------------*/

void ff_memfree (
	void* mblock	/* Pointer to the memory block to free (nothing to do if null) */
)
{
	free(mblock);	/* Free the memory block with POSIX API */
}

#endif



#if FF_FS_REENTRANT	/* Mutal exclusion */

/*------------------------------------------------------------------------*/
/* Create a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount() function to create a new
/  synchronization object for the volume, such as semaphore and mutex.
/  When a 0 is returned, the f_mount() function fails with FR_INT_ERR.
*/

//const osMutexDef_t Mutex[FF_VOLUMES];	/* Table of CMSIS-RTOS mutex */


int ff_cre_syncobj (	/* 1:Function succeeded, 0:Could not create the sync object */
	BYTE vol,			/* Corresponding volume (logical drive number) */
	FF_SYNC_t* sobj		/* Pointer to return the created sync object */
)
{
	/* Win32 */
	*sobj = CreateMutex(NULL, FALSE, NULL);
	return (int)(*sobj != INVALID_HANDLE_VALUE);

	/* uITRON */
//	T_CSEM csem = {TA_TPRI,1,1};
//	*sobj = acre_sem(&csem);
//	return (int)(*sobj > 0);

	/* uC/OS-II */
//	OS_ERR err;
//	*sobj = OSMutexCreate(0, &err);
//	return (int)(err == OS_NO_ERR);

	/* FreeRTOS */
//	*sobj = xSemaphoreCreateMutex();
//	return (int)(*sobj != NULL);

	/* CMSIS-RTOS */
//	*sobj = osMutexCreate(&Mutex[vol]);
//	return (int)(*sobj != NULL);
}


/*------------------------------------------------------------------------*/
/* Delete a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount() function to delete a synchronization
/  object that created with ff_cre_syncobj() function. When a 0 is returned,
/  the f_mount() function fails with FR_INT_ERR.
*/

int ff_del_syncobj (	/* 1:Function succeeded, 0:Could not delete due to an error */
	FF_SYNC_t sobj		/* Sync object tied to the logical drive to be deleted */
)
{
	/* Win32 */
	return (int)CloseHandle(sobj);

	/* uITRON */
//	return (int)(del_sem(sobj) == E_OK);

	/* uC/OS-II */
//	OS_ERR err;
//	OSMutexDel(sobj, OS_DEL_ALWAYS, &err);
//	return (int)(err == OS_NO_ERR);

	/* FreeRTOS */
//  vSemaphoreDelete(sobj);
//	return 1;

	/* CMSIS-RTOS */
//	return (int)(osMutexDelete(sobj) == osOK);
}


/*------------------------------------------------------------------------*/
/* Request Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on entering file functions to lock the volume.
/  When a 0 is returned, the file function fails with FR_TIMEOUT.
*/

int ff_req_grant (	/* 1:Got a grant to access the volume, 0:Could not get a grant */
	FF_SYNC_t sobj	/* Sync object to wait */
)
{
	/* Win32 */
	return (int)(WaitForSingleObject(sobj, FF_FS_TIMEOUT) == WAIT_OBJECT_0);

	/* uITRON */
//	return (int)(wai_sem(sobj) == E_OK);

	/* uC/OS-II */
//	OS_ERR err;
//	OSMutexPend(sobj, FF_FS_TIMEOUT, &err));
//	return (int)(err == OS_NO_ERR);

	/* FreeRTOS */
//	return (int)(xSemaphoreTake(sobj, FF_FS_TIMEOUT) == pdTRUE);

	/* CMSIS-RTOS */
//	return (int)(osMutexWait(sobj, FF_FS_TIMEOUT) == osOK);
}


/*------------------------------------------------------------------------*/
/* Release Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on leaving file functions to unlock the volume.
*/

void ff_rel_grant (
	FF_SYNC_t sobj	/* Sync object to be signaled */
)
{
	/* Win32 */
	ReleaseMutex(sobj);

	/* uITRON */
//	sig_sem(sobj);

	/* uC/OS-II */
//	OSMutexPost(sobj);

	/* FreeRTOS */
//	xSemaphoreGive(sobj);

	/* CMSIS-RTOS */
//	osMutexRelease(sobj);
}

#endif

#if !FF_FS_NORTC

typedef struct
{
  uint8_t WeekDay;  /*!< Specifies the RTC Date WeekDay (not necessary for HAL_RTC_SetDate).
                         This parameter can be a value of @ref RTC_WeekDay_Definitions */

  uint8_t Month;    /*!< Specifies the RTC Date Month (in BCD format).
                         This parameter can be a value of @ref RTC_Month_Date_Definitions */

  uint8_t Date;     /*!< Specifies the RTC Date.
                         This parameter must be a number between Min_Data = 1 and Max_Data = 31 */

  uint8_t Year;     /*!< Specifies the RTC Date Year.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 99 */

} RTC_DateTypeDef;

typedef struct
{
  uint8_t Hours;            /*!< Specifies the RTC Time Hour.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 23 */

  uint8_t Minutes;          /*!< Specifies the RTC Time Minutes.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t Seconds;          /*!< Specifies the RTC Time Seconds.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

} RTC_TimeTypeDef;

DWORD get_fattime(void)
{
  DWORD dw;
  RTC_DateTypeDef date;
  RTC_TimeTypeDef time;
//  
//  rtc_gettime(&date, &time);
    
  dw = (date.Year + 20) << 25; // Year origin from the 1980
  dw |= date.Month << 21;
  dw |= date.Date << 16;
  dw |= time.Hours << 11;
  dw |= time.Minutes << 5;
  dw |= time.Seconds >> 1;
  return dw;
}
#endif

static FATFS fatfs;

void ftp_file_init(void)
{
  char *buffer;
  FRESULT fr;
// W25Qxx_erase_all();
  // 初始化W25Qxx并挂载到FatFs上
  fr = f_mount(&fatfs, "C:\\", 1);
  //fr = FR_NO_FILESYSTEM;
  if (fr == FR_OK)
  {
    // 挂载成功
    printf("Disk is mounted!\n");
  }
  else if (fr == FR_NO_FILESYSTEM)
  {
    // 磁盘未被格式化
    buffer = rt_malloc(FF_MIN_SS);
    if (buffer != NULL)
    {
      fr = f_mkfs("C:\\", FM_FAT, 0, buffer, FF_MIN_SS); // 格式化磁盘
      if (fr == FR_OK)
        printf("Disk is formatted!\n"); // 格式化成功
      else
        printf("Disk is not formatted! fr=%d\n", fr); // 格式化失败
      rt_free(buffer);
    }
    else
      printf("Failed to allocate memory for disk formatting!\n");
  }
  else
  {
    // 挂载失败
    printf("f_mount() failed! fr=%d\n", fr);
  }
}
