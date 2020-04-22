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
*	函 数 名: bsp_CloseAllLed
*	功能说明: 关闭所有LED灯
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: bsp_OpenThreeWhileLed
*	功能说明: 打开3个白色灯，关闭其他颜色灯
*	形    参: 无
*	返 回 值: 无
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
		ledAppProc.delay = bsp_GetRunTime() ;
		
		ledAppProc.ledAppState = ledAppState;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_LedAppProc
*	功能说明: LED处理函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_LedAppProc(void)
{
//	if(!ledAppProc.isRunning)
//		return;
	
	
	if(ledAppProc.ledAppState == LED_DEFAULT_STATE)       /*默认状态,不处理*/
	{
		
	}
	else if(ledAppProc.ledAppState == THREE_WHITE_TOOGLE) /*三颗白色灯闪烁*/
	{
		if(bsp_GetRunTime() - ledAppProc.delay >= 360)
		{
			bsp_LedToggle(LED_LOGO_CLEAN);
			bsp_LedToggle(LED_LOGO_POWER);
			bsp_LedToggle(LED_LOGO_CHARGE);
			
			ledAppProc.delay = bsp_GetRunTime();
		}
	}
	
	
}

