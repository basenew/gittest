#ifndef __W25Qxx_H
#define __W25Qxx_H
#include "stdint.h"
#define W25Qxx_STATUS_BUSY 0x01
#define W25Qxx_STATUS_WEL 0x02 // Write Enable Latch
#define W25Qxx_STATUS2_QE 0x02 // Quad Enable


void W25Qxx_EraseSector(uint16_t sector);
void W25Qxx_Init(void);
void W25Qxx_erase_all(void);
void W25Qxx_ProgramPage(uint32_t addr, const void *data, int len);
void W25Qxx_Read(uint32_t addr, void *data, int len);
uint8_t W25Qxx_ReadID(uint8_t *pm);

#endif
