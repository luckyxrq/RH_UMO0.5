#ifndef __BSP_STFLASH_H
#define __BSP_STFLASH_H

#include <stdbool.h>

/*�ڲ�FLASH��С����λKB��*/
#define STM32_FLASH_SIZE 64

/*PAGE��С����λ�ֽڣ�*/
#if STM32_FLASH_SIZE<256
#define STM_PAGE_SIZE      1024
#define FLASH_BUF_SIZE     512
#else 
#define STM_PAGE_SIZE      2048
#define FLASH_BUF_SIZE     1024
#endif	

/*STM32 FLASH����ʼ��ַ*/
#define STM32_FLASH_BASE      0x08000000

/*ҳ��ţ���С�����ֵ*/
#define MIN_PAGE_INDEX      0
#define MAX_PAGE_INDEX      (STM32_FLASH_SIZE / (STM_PAGE_SIZE/1024) - 1)

/*FLASH BUF���������飬���ڲ��ԣ�103C8T6ÿ��ҳֻ��1K*/
static uint16_t flashBuf[FLASH_BUF_SIZE];

bool bsp_FlashWritePage(uint16_t pageIndex , uint16_t buf[] , uint16_t len);
bool bsp_FlashReadPage(uint16_t pageIndex , uint16_t buf[] , uint16_t len);
void bsp_StFlashTest(uint16_t pageIndex);

#endif

