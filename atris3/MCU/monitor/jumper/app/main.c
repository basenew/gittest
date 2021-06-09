#include "bitmap.h"
#include "delay.h"
#include "24cxx.h"
#include "rt_fota_flag.h"
#include "../../ubt_common.h"

/*jumper version*/
#define JUMPER_VER 0x01

#define JUMPER_ADDR DF_JUMPER_LINKER_ADDR
#define BOOTLOADER_ADDR DF_BOOTLOADER_LINKER_ADDR
#define APP_ADDR DF_APP_LINKER_ADDR

#define FLAGS_POS_OFFSET 0

extern void usart_init(void);
extern void usart_puts(USART_TypeDef *USARTx, volatile char *s);

uint8_t version_check(void)
{
	uint8_t temp = 0xFF;
	temp = AT24CXX_ReadOneByte(FLAGS_POS_OFFSET);
	if (temp == JUMPER_VER)
		return 0;
	else
	{
		AT24CXX_WriteOneByte(FLAGS_POS_OFFSET, JUMPER_VER);
		temp = AT24CXX_ReadOneByte(FLAGS_POS_OFFSET);
		if (temp == JUMPER_VER)
			return 0;
	}
	return 1;
}

int32_t jump_to(uint32_t _addr)
{
    typedef void (*rt_fota_app_func)(void); 
    rt_fota_app_func app_func = (void*)0;

    if (((*(__IO uint32_t *)_addr) & 0x2ffe0000) != 0x20000000)  {
        usart_puts(USART1, "[jumper]address is illegal.\r\n");
        return -1;
    }

    __disable_irq();
    app_func = (rt_fota_app_func)*(__IO uint32_t *)(_addr + 4);
    /* Configure main stack */
    __set_MSP(*(__IO uint32_t *)_addr);
    /* jump to application */
    app_func();  
    while(1);
}

static fota_flags_t fota_flags;

int main(void)
{
	delay_init(168);

    extern void vcc_3v3_Init(void);
    vcc_3v3_Init();
    
    usart_init();

	AT24CXX_Init();	
    
	if (version_check() != 0) {
        usart_puts(USART1, "[jumper]to bootloader, because EEPROM check error.\r\n");
        jump_to(BOOTLOADER_ADDR);
    }

    AT24CXX_Read(FLAGS_POS_OFFSET, (uint8_t*)(&fota_flags), sizeof(fota_flags_t));

    /*check if the flags is initialized or not*/
    if (fota_flags.initialized != FLAG_INIT) {
        usart_puts(USART1, "[jumper]to bootloader, because FLAGS is not initialized yet.\r\n");
        jump_to(BOOTLOADER_ADDR);
    }
    
    if (fota_flags.uflag.sflag.jump_to_where == FLAG_TO_BL) {
        usart_puts(USART1, "[jumper]to bootloader.\r\n");
        jump_to(BOOTLOADER_ADDR);
    } else {
        usart_puts(USART1, "[jumper]to app.\r\n");
        if (jump_to(APP_ADDR) < 0) {
            usart_puts(USART1, "[jumper]to bootloader.\r\n");
            jump_to(BOOTLOADER_ADDR);
        }
    }
    while (1);
}

