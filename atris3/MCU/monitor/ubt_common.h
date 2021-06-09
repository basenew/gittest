#ifndef UBT_COMMON_H__
#define UBT_COMMON_H__

#define DF_COMPILE_RELEASE 
//#define DF_COMPILE_DEBUG  

#define DF_APP_SOFTWARE_VERSION "0.9.0.beta2"

#define DF_HARDWARE_VERSION 1

#define DF_COMPATIBLE_HARDWARE_VERSION 0

#define DF_BL_SOFTWARE_VERSION "1.1.0.beta"



#ifdef DF_COMPILE_RELEASE

#define DF_JUMPER_LINKER_ADDR 0x08000000
#define DF_JUMPER_LINKER_SIZE 16k

#define DF_BOOTLOADER_LINKER_ADDR 0x08004000
#define DF_BOOTLOADER_LINKER_SIZE 112k

#define DF_APP_LINKER_ADDR 0x08020000
#define DF_APP_LINKER_SIZE  	896k

#else

#define DF_JUMPER_LINKER_ADDR 0x08000000
#define DF_JUMPER_LINKER_SIZE 16k

#define DF_BOOTLOADER_LINKER_ADDR 0x08000000
#define DF_BOOTLOADER_LINKER_SIZE 1024k

#define DF_APP_LINKER_ADDR 0x08000000
#define DF_APP_LINKER_SIZE  	1024k

#endif


#endif