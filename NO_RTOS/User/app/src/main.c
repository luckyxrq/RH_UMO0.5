#include "bsp.h"


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
	
	
	
//	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(20));
//	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(20));

//	bsp_IRD_StartWork();
	
	bsp_LedOn(LED_LOGO_CLEAN);
	bsp_LedOn(LED_LOGO_POWER);
	bsp_LedOn(LED_LOGO_CHARGE);
	bsp_LedOff(LED_COLOR_YELLOW);
	bsp_LedOff(LED_COLOR_GREEN);
	bsp_LedOff(LED_COLOR_RED);
	
	
	/* �������ѭ�� */
	while (1)
	{
	
		if(tick % 500 == 0)
		{
			bsp_PrintIR_Rev();
		}
		
		if(tick % 10 == 0)
		{
			bsp_PositionUpdate();
		}
		
		/*�������ݲ��޸�*/
		++tick;
		bsp_DelayMS(1);
	}
}


