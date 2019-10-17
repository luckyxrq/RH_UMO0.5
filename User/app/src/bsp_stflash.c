#include "bsp.h"

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
	for(i=0;i<FLASH_BUF_SIZE;i++)
	{
		flashBuf[i] = i;
	}
	
	/*д���ڲ�FLASH*/
	bsp_FlashWritePage(pageIndex,flashBuf,FLASH_BUF_SIZE);
	
	/*������飬ʹ��memset��գ��޷���512�����������գ�ԭ��֪*/
	for(i=0;i<FLASH_BUF_SIZE;i++)
	{
		flashBuf[i] = 0;
	}

	/*��ȡ�ڲ�FLASH*/
	bsp_FlashReadPage(pageIndex,flashBuf,FLASH_BUF_SIZE);
	
	/*��ӡ��Ϣ*/
	for(i=0;i<FLASH_BUF_SIZE;i++)
	{
		DEBUG("flashBuf[%d]:%d\r\n",i,flashBuf[i]);
	}
	
	DEBUG("*********************************************\r\n\r\n");
}

