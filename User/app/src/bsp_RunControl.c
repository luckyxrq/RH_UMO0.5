#include "bsp.h"

#define MAX_LINK_TOGGLE_CNT      60//20


uint8_t  work_mode = smart;
uint8_t  work_switch_go = false;
uint8_t  work_status = idle;
uint8_t  wifi_link_complete = 0;

typedef struct
{
	bool isRunning;
	uint32_t delay;
	uint8_t action;
	
	uint8_t nowLinkToggleCnt;
	
	LedAppState ledAppState;
}LedAppProc;

static LedAppProc ledAppProc;





static KEY_STATE key_state = eKEY_NONE;

void bsp_SetLastKeyState(KEY_STATE state)
{
	key_state = state;
}

KEY_STATE bsp_GetLastKeyState(void)
{
	return key_state;
}


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
		ledAppProc.nowLinkToggleCnt = 0 ;
		
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
	
	for(i=0;i<3;i++)
	{
		bsp_LedToggle(LED_LOGO_CLEAN);
		bsp_LedToggle(LED_LOGO_POWER);
		bsp_LedToggle(LED_LOGO_CHARGE);
		
		bsp_DelayMS(500);
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

	if(ledAppProc.ledAppState == LED_DEFAULT_STATE)       /*Ĭ��״̬,������*/
	{
		work_status = idle;
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
		work_status = idle;
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOn(LED_LOGO_CHARGE);
		bsp_LedOff(LED_COLOR_YELLOW);
		bsp_LedOff(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
	}
	else if(ledAppProc.ledAppState == AT_CHARGING) /*�����*/
	{
		work_status = charging;
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOff(LED_LOGO_CHARGE);
		bsp_LedOn(LED_COLOR_YELLOW);
		bsp_LedOff(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
		
		/*������һ�ΰ���ֵ*/
		bsp_SetLastKeyState(eKEY_NONE);
	}
	else if(ledAppProc.ledAppState == AT_CHARGE_DONE) /*������*/
	{
		work_status = charge_done;
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOff(LED_LOGO_CHARGE);
		bsp_LedOff(LED_COLOR_YELLOW);
		bsp_LedOn(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
	}
	else if(ledAppProc.ledAppState == AT_SEARCH_CHARGE)
	{
		work_status = goto_charge;
		if(xTaskGetTickCount() - ledAppProc.delay >= 360)
		{
			bsp_LedToggle(LED_LOGO_CHARGE);
			
			ledAppProc.delay = xTaskGetTickCount();
		}
	}
	else if(ledAppProc.ledAppState == AT_CLEAN)
	{
		if(work_mode == smart )work_status = smart_clean; 
		if(work_mode == wall_follow )work_status = wall_clean;
		
		if(xTaskGetTickCount() - ledAppProc.delay >= 360)
		{
			bsp_LedToggle(LED_LOGO_CLEAN);
			
			ledAppProc.delay = xTaskGetTickCount();
		}
	}
	else if(ledAppProc.ledAppState == AT_LINK)
	{
		if(xTaskGetTickCount() - ledAppProc.delay >= 360)
		{
			bsp_LedToggle(LED_LOGO_CLEAN);
			bsp_LedToggle(LED_LOGO_CHARGE);
			bsp_LedOff(LED_LOGO_POWER);
			bsp_LedOff(LED_COLOR_YELLOW);
			bsp_LedOff(LED_COLOR_GREEN);
			bsp_LedOff(LED_COLOR_RED);
			
			ledAppProc.delay = xTaskGetTickCount();
			
			if(++ledAppProc.nowLinkToggleCnt >= MAX_LINK_TOGGLE_CNT)
			{
				ledAppProc.ledAppState = AT_LINKDONE;
			}
			
			if(wifi_link_complete)
			{
				wifi_link_complete = 0;
				ledAppProc.ledAppState = AT_LINKDONE;
				bsp_SperkerPlay(Song27);
			}
		}
	}
	else if(ledAppProc.ledAppState == AT_LINKDONE)
	{
		bsp_OpenThreeWhileLed();
		ledAppProc.ledAppState = LED_DEFAULT_STATE;
	}
}

LedAppState bsp_GetLedAppState(void)
{
	return ledAppProc.ledAppState;
}


