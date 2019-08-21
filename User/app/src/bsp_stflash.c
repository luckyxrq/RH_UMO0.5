#include "bsp.h"

/*内部FLASH大小（单位KB）*/
#define STM32_FLASH_SIZE 512

/*PAGE大小（单位字节）*/
#if STM32_FLASH_SIZE<256
#define STM_PAGE_SIZE      1024
#else 
#define STM_PAGE_SIZE      2048
#endif	

/*STM32 FLASH的起始地址*/
#define STM32_FLASH_BASE      0x08000000

/*页序号，最小及最大值*/
#define MIN_PAGE_INDEX      0
#define MAX_PAGE_INDEX      (STM32_FLASH_SIZE / (STM_PAGE_SIZE/1024) - 1)


/*
*********************************************************************************************************
*	函 数 名: bsp_FlashWrite
*	功能说明: 写一个页
*	形    参: 页序号，缓冲区地址，要写入的半字（2个字节）数
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_FlashWritePage(uint16_t pageIndex , uint16_t buf[] , uint16_t len)
{
	uint16_t i = 0 ;
	uint32_t addr = pageIndex*STM_PAGE_SIZE+STM32_FLASH_BASE ;
	
	if(pageIndex > MAX_PAGE_INDEX)
	{
		return false;
	}
	
	/*解锁*/
	FLASH_Unlock();
	/*擦除页*/
	FLASH_ErasePage(addr);
	/*写，每次必须写半字*/
	for(i=0;i<len;i++)
	{
		FLASH_ProgramHalfWord(addr,buf[i]);
		addr += 2;
	}
	/*上锁*/
	FLASH_Lock();
	
	return true;
}



/*
*********************************************************************************************************
*	函 数 名: bsp_FlashWrite
*	功能说明: 写一个页
*	形    参: 页序号，缓冲区地址，要读取的半字（2个字节）数
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_FlashReadPage(uint16_t pageIndex , uint16_t buf[] , uint16_t len)
{
	uint16_t i = 0 ;
	uint32_t addr = pageIndex*STM_PAGE_SIZE+STM32_FLASH_BASE ;
	
	if(pageIndex > MAX_PAGE_INDEX)
	{
		return false;
	}
	
    for(i=0;i<len;i++)
    {
        buf[i] = *(volatile uint16_t*)addr;
        addr += 2;
    }
	
	return true;
}

