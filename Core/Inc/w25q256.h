/**
 * @file w25q256.h
 * @brief W25Q256 32MB QSPI NOR Flash 低层驱动
 */
#ifndef __W25Q256_H__
#define __W25Q256_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "quadspi.h"
#include <stdbool.h>

#define W25Q256_SECTOR_SIZE      4096U
#define W25Q256_BLOCK_SIZE        65536U
#define W25Q256_PAGE_SIZE         256U
#define W25Q256_CAPACITY          33554432UL  /* 32 MB */
#define W25Q256_SECTOR_COUNT      (W25Q256_CAPACITY / W25Q256_SECTOR_SIZE)
#define W25Q256_MANUFACTURER_ID   0xEF
#define W25Q256_DEVICE_ID         0x4019

bool        W25Q256_Init(void);
bool        W25Q256_Reset(void);
uint32_t    W25Q256_ReadID(void);
bool        W25Q256_IsPresent(void);
uint32_t    W25Q256_GetCapacity(void);
const char* W25Q256_GetName(void);

bool W25Q256_EraseSector(uint32_t addr);
bool W25Q256_EraseBlock(uint32_t addr);
bool W25Q256_ChipErase(void);

bool W25Q256_Read(uint32_t addr, uint8_t *buf, uint32_t len);
bool W25Q256_Write(uint32_t addr, const uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __W25Q256_H__ */
