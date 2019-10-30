#include "bsp.h"

#define PUMP_OPEN_MS       2000   /*ˮ�ÿ�ʱ��*/
#define PUMP_CLOSE_MS      4000   /*ˮ�ù�ʱ��*/

/*ˮ�ÿ���*/
#define  bsp_PumpOpen()   TIM_SetCompare3(TIM4,CONSTANT_HIGH_PWM)
#define  bsp_PumpClose()  TIM_SetCompare3(TIM4,0)

typedef struct
{
	bool isRunning;
	uint8_t action;
	uint32_t delay;
	
}Pump;

static Pump pump;

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitPump
*	����˵��: ��ʼ��ˮ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitPump(void)
{
	/*
		ˮ�õĿ��ƿڣ�ʹ�õ���ɨ�ػ��ı�ˢ���ƿڣ�
		��ˢ���ƿ�Ĭ���Ǹߵ�ƽ�����Ǹߵ�ƽˮ�þͶ��ˣ�
		���Գ�ʼ����ʱ��͵ð�ˮ�ùص�
	*/
	bsp_PumpClose();
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StartPumpRun
*	����˵��: ����ˮ������ѭ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartPumpRun(void)
{
	pump.action = 0 ;
	pump.delay = 0 ;
	pump.isRunning = true;
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StopPumpRun
*	����˵��: �ر�ˮ������ѭ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StopPumpRun(void)
{
	pump.isRunning = false;
	pump.action = 0 ;
	pump.delay = 0 ;
	
	bsp_PumpClose();
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_PumpRun
*	����˵��: ˮ������ѭ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_PumpRun(void)
{
	if(!pump.isRunning)
		return;
	
	switch(pump.action)
	{
		case 0: /*��ˮ��*/
		{
			bsp_PumpOpen();
			pump.delay = xTaskGetTickCount();
			pump.action++;
		}break;
		
		case 1: /*����ʱ�� PUMP_OPEN_MS ��Ȼ���ˮ��*/
		{
			if(xTaskGetTickCount() - pump.delay >= PUMP_OPEN_MS)
			{
				bsp_PumpClose();
				pump.delay = xTaskGetTickCount();
				pump.action++;
			}
		}break;
		
		case 2: /*����ʱ�� bsp_PumpClose ��Ȼ��ص�ѭ���Ŀ�ʼ*/
		{
			if(xTaskGetTickCount() - pump.delay >= PUMP_CLOSE_MS)
			{
				pump.action = 0 ;
			}
		}break;
	}
}
