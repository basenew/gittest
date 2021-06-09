BSP_ROOT ?= E:/dev_ubt/atris_mcu/_git/atris3/MCU/chassis_controller/APP
RTT_ROOT ?= E:/dev_ubt/atris_mcu/_git/atris3/MCU/rt-thread-master

CROSS_COMPILE ?=D:\\dev\\arm\\rtt\\env\\tools\\gnu_gcc\\arm_gcc\\mingw\\bin\\arm-none-eabi-

CFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O2
AFLAGS := -c -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb 
LFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=rt-thread.map,-cref,-u,Reset_Handler -T board/linker_scripts/link.lds
CXXFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O2

CPPPATHS :=-I"E:\dev_ubt\atris_mcu\_git\atris3\MCU\libraries\HAL_Drivers", \
	-I"E:\dev_ubt\atris_mcu\_git\atris3\MCU\libraries\HAL_Drivers\config", \
	-I"E:\dev_ubt\atris_mcu\_git\atris3\MCU\libraries\HAL_Drivers\drv_flash", \
	-I"E:\dev_ubt\atris_mcu\_git\atris3\MCU\libraries\STM32F4xx_HAL\CMSIS\Device\ST\STM32F4xx\Include", \
	-I"E:\dev_ubt\atris_mcu\_git\atris3\MCU\libraries\STM32F4xx_HAL\CMSIS\Include", \
	-I"E:\dev_ubt\atris_mcu\_git\atris3\MCU\libraries\STM32F4xx_HAL\STM32F4xx_HAL_Driver\Inc", \
	-I"E:\dev_ubt\atris_mcu\_git\atris3\MCU\tinyros_client_library\components", \
	-I$(BSP_ROOT) \
	-I$(BSP_ROOT)\applications \
	-I$(BSP_ROOT)\applications\battery \
	-I$(BSP_ROOT)\applications\fota \
	-I$(BSP_ROOT)\applications\log \
	-I$(BSP_ROOT)\applications\modules \
	-I$(BSP_ROOT)\board \
	-I$(BSP_ROOT)\board\ports \
	-I$(BSP_ROOT)\chassis\MOTOR_driver \
	-I$(BSP_ROOT)\chassis\STM32F4xx_StdPeriph_Driver\inc \
	-I$(BSP_ROOT)\chassis\controller \
	-I$(BSP_ROOT)\chassis\ftp \
	-I$(BSP_ROOT)\chassis\httpd \
	-I$(BSP_ROOT)\packages\EasyFlash-v4.1.0\inc \
	-I$(BSP_ROOT)\packages\at24cxx-latest \
	-I$(BSP_ROOT)\packages\fal-v0.5.0\inc \
	-I$(BSP_ROOT)\packages\littlefs-v2.1.4 \
	-I$(BSP_ROOT)\packages\ota_downloader-latest \
	-I$(BSP_ROOT)\packages\quick_led-latest\inc \
	-I$(BSP_ROOT)\packages\rtc \
	-I$(BSP_ROOT)\packages\webclient-v2.1.2\inc \
	-I$(RTT_ROOT)\components\cplusplus \
	-I$(RTT_ROOT)\components\dfs\filesystems\devfs \
	-I$(RTT_ROOT)\components\dfs\include \
	-I$(RTT_ROOT)\components\drivers\include \
	-I$(RTT_ROOT)\components\drivers\sensors \
	-I$(RTT_ROOT)\components\drivers\spi \
	-I$(RTT_ROOT)\components\drivers\spi\sfud\inc \
	-I$(RTT_ROOT)\components\finsh \
	-I$(RTT_ROOT)\components\libc\compilers\newlib \
	-I$(RTT_ROOT)\components\net\lwip-2.0.2\src \
	-I$(RTT_ROOT)\components\net\lwip-2.0.2\src\arch\include \
	-I$(RTT_ROOT)\components\net\lwip-2.0.2\src\include \
	-I$(RTT_ROOT)\components\net\lwip-2.0.2\src\include\ipv4 \
	-I$(RTT_ROOT)\components\net\lwip-2.0.2\src\include\netif \
	-I$(RTT_ROOT)\components\net\netdev\include \
	-I$(RTT_ROOT)\components\net\sal_socket\impl \
	-I$(RTT_ROOT)\components\net\sal_socket\include \
	-I$(RTT_ROOT)\components\net\sal_socket\include\dfs_net \
	-I$(RTT_ROOT)\components\net\sal_socket\include\socket \
	-I$(RTT_ROOT)\components\net\sal_socket\include\socket\sys_socket \
	-I$(RTT_ROOT)\components\utilities\ulog \
	-I$(RTT_ROOT)\components\utilities\ymodem \
	-I$(RTT_ROOT)\include \
	-I$(RTT_ROOT)\libcpu\arm\common \
	-I$(RTT_ROOT)\libcpu\arm\cortex-m4 

DEFINES := -DHAVE_CCONFIG_H -DLFS_CONFIG=lfs_config.h -DRT_USING_NEWLIB -DSTM32F407xx -DUSE_HAL_DRIVER
