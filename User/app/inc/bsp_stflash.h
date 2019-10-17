#ifndef __BSP_STFLASH_H
#define __BSP_STFLASH_H

#include <stdbool.h>

/*内部FLASH大小（单位KB）*/
#define STM32_FLASH_SIZE 64

/*PAGE大小（单位字节）*/
#if STM32_FLASH_SIZE<256
#define STM_PAGE_SIZE      1024
#define FLASH_BUF_SIZE     512
#else 
#define STM_PAGE_SIZE      2048
#define FLASH_BUF_SIZE     1024
#endif	

/*STM32 FLASH的起始地址*/
#define STM32_FLASH_BASE      0x08000000

/*页序号，最小及最大值*/
#define MIN_PAGE_INDEX      0
#define MAX_PAGE_INDEX      (STM32_FLASH_SIZE / (STM_PAGE_SIZE/1024) - 1)

/*FLASH BUF，半字数组，用于测试，103C8T6每个页只有1K*/
static uint16_t flashBuf[FLASH_BUF_SIZE];

bool bsp_FlashWritePage(uint16_t pageIndex , uint16_t buf[] , uint16_t len);
bool bsp_FlashReadPage(uint16_t pageIndex , uint16_t buf[] , uint16_t len);
void bsp_StFlashTest(uint16_t pageIndex);

#endif

