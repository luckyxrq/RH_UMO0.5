#include "bsp.h"


static void bsp_KeyProc(void);

/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: c�������
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
int main(void)
{
	uint32_t tick = 0 ;
	/*
		ST�̼����е������ļ��Ѿ�ִ���� SystemInit() �������ú����� system_stm32f4xx.c �ļ�����Ҫ������
	����CPUϵͳ��ʱ�ӣ��ڲ�Flash����ʱ������FSMC�����ⲿSRAM
	*/
	bsp_Init();		/* Ӳ����ʼ�� */
	
	
	
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));

//	bsp_IRD_StartWork();
	
	bsp_SetLedState(THREE_WHITE_TOOGLE);
	
	
	/* �������ѭ�� */
	while (1)
	{
	
		bsp_PositionUpdate();
		bsp_LedAppProc();
		bsp_KeyProc();
		bsp_SearchChargePile();
		
		/*�������ݲ��޸�*/
		++tick;
		bsp_DelayMS(1);
	}
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_KeyProc
*	����˵��: ����������	  			  
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_KeyProc(void)
{
	uint8_t ucKeyCode;	
	
	ucKeyCode = bsp_GetKey();
	if (ucKeyCode > 0)
	{
		/* �м����� */
		switch (ucKeyCode)
		{
			case KEY_DOWN_POWER:
			{
				DEBUG("��Դ��������\r\n");

			}break;
				
			case KEY_DOWN_CHARGE:
			{
				DEBUG("��簴������\r\n");

			}break;
				
			case KEY_DOWN_CLEAN:	
			{
				DEBUG("��ɨ��������\r\n");

			}break;

			case KEY_LONG_POWER:
			{
				DEBUG("��Դ��������\r\n");
				
				
			}break;
			
			case KEY_LONG_CHARGE:
			{
				DEBUG("��簴������\r\n");
				
				
			}break;
			
			case KEY_LONG_CLEAN:
			{
				DEBUG("��ɨ��������\r\n");
				
				
			}break;
			
			case KEY_9_DOWN:
			{
				
				
			}break;
			
			
			case KEY_10_DOWN:
			{
				DEBUG("����������ͬʱ��������ɨ\r\n");

			}break;
		}   
	}
}


