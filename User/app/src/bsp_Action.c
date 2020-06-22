#include "bsp.h"

#define FREE_STATE      (!bsp_IsTouchChargePile() &&  bsp_GetLedAppState() != AT_CLEAN && bsp_GetLedAppState() != AT_SEARCH_CHARGE &&bsp_GetLedAppState() != THREE_WHITE_TOOGLE &&bsp_GetLedAppState() != AT_LINK)
#define FREE_LONG_TIME_SLEEP     (5*60*1000) /*空闲5分钟 而且不在桩上则进入休眠模式*/


typedef struct
{
	bool isRunning;
	uint32_t delay;
	uint32_t action;
	
	uint32_t lastFreeTime;
}SleepProc;

static SleepProc sleepProc;


void bsp_StartSleepProc(void)
{
	sleepProc.action = 0 ;
	sleepProc.delay = 0 ;

	sleepProc.lastFreeTime = 0;
	
	sleepProc.isRunning = true;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_SleepProc
*	功能说明: 常时间没有清扫或者回充，自动进入休眠模式
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SleepProc(void)
{
	if(!sleepProc.isRunning)
	{
		return;
	}
	
	switch(sleepProc.action)
	{
		case 0:
		{
			if(FREE_STATE)
			{
				sleepProc.lastFreeTime = xTaskGetTickCount();
				++sleepProc.action;
			}
		}break;
		
		case 1:
		{
			if(!FREE_STATE)
			{
				sleepProc.action = 0 ;
			}
			else
			{
				if(xTaskGetTickCount() - sleepProc.lastFreeTime >= FREE_LONG_TIME_SLEEP)
				{
					bsp_PutKey(KEY_LONG_POWER); /*模拟按键按下进入休眠模式*/
					++sleepProc.action;
				}
			}
		}break;
		
		default:break;
	}
	
	
	
}

