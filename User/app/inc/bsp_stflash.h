#ifndef __BSP_STFLASH_H
#define __BSP_STFLASH_H

bool bsp_FlashWritePage(uint16_t pageIndex , uint16_t buf[] , uint16_t len);
bool bsp_FlashReadPage(uint16_t pageIndex , uint16_t buf[] , uint16_t len);
void bsp_StFlashTest(uint16_t pageIndex);

#endif

