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
		ledAppProc.delay = xTaskGetTickCount() ;
		
		ledAppProc.ledAppState = ledAppState;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_PowerOnLedProc
*	功能说明: 开机的LED闪烁，停住闪烁
*	形    参: 无
*	返 回 值: 无
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
		if(xTaskGetTickCount() - ledAppProc.delay >= 360)
		{
			bsp_LedToggle(LED_LOGO_CLEAN);
			bsp_LedToggle(LED_LOGO_POWER);
			bsp_LedToggle(LED_LOGO_CHARGE);
			
			ledAppProc.delay = xTaskGetTickCount();
		}
	}
	else if(ledAppProc.ledAppState == THREE_WHITE_ON) /*亮3颗白色LED*/
	{
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOn(LED_LOGO_CHARGE);
		bsp_LedOff(LED_COLOR_YELLOW);
		bsp_LedOff(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
	}
	else if(ledAppProc.ledAppState == AT_CHARGING) /*充电中*/
	{
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOff(LED_LOGO_CHARGE);
		bsp_LedOn(LED_COLOR_YELLOW);
		bsp_LedOff(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
	}
	else if(ledAppProc.ledAppState == AT_CHARGE_DONE) /*充电中*/
	{
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOff(LED_LOGO_CHARGE);
		bsp_LedOff(LED_COLOR_YELLOW);
		bsp_LedOn(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
	}
}



