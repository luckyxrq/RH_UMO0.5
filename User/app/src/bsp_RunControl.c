#include "bsp.h"

typedef struct
{
	bool isRunning;
	uint32_t delay;
	uint8_t action;
	
	LedAppState ledAppState;
}LedAppProc;

static LedAppProc ledAppProc;

/*
*********************************************************************************************************
*	�� �� ��: bsp_CloseAllLed
*	����˵��: �ر�����LED��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_CloseAllLed(void)
{
	bsp_LedOff(LED_LOGO_CLEAN);
	bsp_LedOff(LED_LOGO_POWER);
	bsp_LedOff(LED_LOGO_CHARGE);
	bsp_LedOff(LED_COLOR_YELLOW);
	bsp_LedOff(LED_COLOR_GREEN);
	bsp_LedOff(LED_COLOR_RED);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_OpenThreeWhileLed
*	����˵��: ��3����ɫ�ƣ��ر�������ɫ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_OpenThreeWhileLed(void)
{
	bsp_LedOn(LED_LOGO_CLEAN);
	bsp_LedOn(LED_LOGO_POWER);
	bsp_LedOn(LED_LOGO_CHARGE);
	bsp_LedOff(LED_COLOR_YELLOW);
	bsp_LedOff(LED_COLOR_GREEN);
	bsp_LedOff(LED_COLOR_RED);
}

void bsp_SetLedState(LedAppState ledAppState)
{
	if(ledAppState != ledAppProc.ledAppState)
	{
		ledAppProc.delay = xTaskGetTickCount() ;
		
		ledAppProc.ledAppState = ledAppState;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_PowerOnLedProc
*	����˵��: ������LED��˸��ͣס��˸
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_PowerOnLedProc(void)
{
	uint8_t i = 0;
	
	bsp_LedOn(LED_LOGO_CLEAN);
	bsp_LedOn(LED_LOGO_POWER);
	bsp_LedOn(LED_LOGO_CHARGE);
	
	for(i=0;i<5;i++)
	{
		bsp_LedToggle(LED_LOGO_CLEAN);
		bsp_LedToggle(LED_LOGO_POWER);
		bsp_LedToggle(LED_LOGO_CHARGE);
		
		bsp_DelayMS(800);
	}
	
	bsp_LedOn(LED_LOGO_CLEAN);
	bsp_LedOn(LED_LOGO_POWER);
	bsp_LedOn(LED_LOGO_CHARGE);
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_LedAppProc
*	����˵��: LED������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_LedAppProc(void)
{
//	if(!ledAppProc.isRunning)
//		return;
	
	
	if(ledAppProc.ledAppState == LED_DEFAULT_STATE)       /*Ĭ��״̬,������*/
	{
		
	}
	else if(ledAppProc.ledAppState == THREE_WHITE_TOOGLE) /*���Ű�ɫ����˸*/
	{
		if(xTaskGetTickCount() - ledAppProc.delay >= 360)
		{
			bsp_LedToggle(LED_LOGO_CLEAN);
			bsp_LedToggle(LED_LOGO_POWER);
			bsp_LedToggle(LED_LOGO_CHARGE);
			
			ledAppProc.delay = xTaskGetTickCount();
		}
	}
	else if(ledAppProc.ledAppState == THREE_WHITE_ON) /*��3�Ű�ɫLED*/
	{
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOn(LED_LOGO_CHARGE);
		bsp_LedOff(LED_COLOR_YELLOW);
		bsp_LedOff(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
	}
	else if(ledAppProc.ledAppState == AT_CHARGING) /*�����*/
	{
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOff(LED_LOGO_CHARGE);
		bsp_LedOn(LED_COLOR_YELLOW);
		bsp_LedOff(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
	}
	else if(ledAppProc.ledAppState == AT_CHARGE_DONE) /*�����*/
	{
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOff(LED_LOGO_CHARGE);
		bsp_LedOff(LED_COLOR_YELLOW);
		bsp_LedOn(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
	}
}



