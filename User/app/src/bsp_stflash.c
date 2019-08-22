#include "bsp.h"

/*�ڲ�FLASH��С����λKB��*/
#define STM32_FLASH_SIZE 512

/*PAGE��С����λ�ֽڣ�*/
#if STM32_FLASH_SIZE<256
#define STM_PAGE_SIZE      1024
#else 
#define STM_PAGE_SIZE      2048
#endif	

/*STM32 FLASH����ʼ��ַ*/
#define STM32_FLASH_BASE      0x08000000

/*ҳ��ţ���С�����ֵ*/
#define MIN_PAGE_INDEX      0
#define MAX_PAGE_INDEX      (STM32_FLASH_SIZE / (STM_PAGE_SIZE/1024) - 1)

/*FLASH BUF���������飬���ڲ���*/
static uint16_t flashBuf[1024];


/*
*********************************************************************************************************
*	�� �� ��: bsp_FlashWrite
*	����˵��: дһ��ҳ
*	��    ��: ҳ��ţ���������ַ��Ҫд��İ��֣�2���ֽڣ���
*	�� �� ֵ: ��
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
	
	/*����*/
	FLASH_Unlock();
	/*����ҳ*/
	FLASH_ErasePage(addr);
	/*д��ÿ�α���д����*/
	for(i=0;i<len;i++)
	{
		FLASH_ProgramHalfWord(addr,buf[i]);
		addr += 2;
	}
	/*����*/
	FLASH_Lock();
	
	return true;
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_FlashWrite
*	����˵��: дһ��ҳ
*	��    ��: ҳ��ţ���������ַ��Ҫ��ȡ�İ��֣�2���ֽڣ���
*	�� �� ֵ: ��
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


/*
*********************************************************************************************************
*	�� �� ��: bsp_StFlashTest
*	����˵��: FLASH���Ժ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StFlashTest(uint16_t pageIndex)
{
	uint16_t i = 0 ;

	/*������������*/
	for(i=0;i<1024;i++)
	{
		flashBuf[i] = i;
	}
	
	/*д���ڲ�FLASH*/
	bsp_FlashWritePage(pageIndex,flashBuf,1024);
	
	/*������飬ʹ��memset��գ��޷���512�����������գ�ԭ��֪*/
	for(i=0;i<1024;i++)
	{
		flashBuf[i] = 0;
	}

	/*��ȡ�ڲ�FLASH*/
	bsp_FlashReadPage(pageIndex,flashBuf,1024);
	
	/*��ӡ��Ϣ*/
	for(i=0;i<1024;i++)
	{
		DEBUG("flashBuf[%d]:%d\r\n",i,flashBuf[i]);
	}
	
	DEBUG("*********************************************\r\n\r\n");
}

