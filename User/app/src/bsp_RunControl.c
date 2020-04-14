#include "bsp.h"

#define POWER_ON_LED_TIME_INTERVAL            500
#define POWER_ON_LED_MAX_TIMES                5
#define POWER_ON_LED_ON_CONSTANT_MAX_TIME     10*60*1000     

#define WIFI_SMART_CONFIG_TOGGLE_TIMES        20

typedef struct
{
	/*上电闪烁10S，500MS闪烁一次，共20次*/
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
	
	/*闪烁次数计数*/
	volatile uint8_t times;
}PowerOnToggle;

static PowerOnToggle powerOnToggle;
static bool isSelfCheckingReady = false;
static bool isFormSleep = false;

/*
*********************************************************************************************************
*	函 数 名: isCleanCarFromSleep
*	功能说明: 返回是否此次是从睡眠模式醒过来的
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
bool isCleanCarFromSleep(void)
{
	return isFormSleep;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_setCleanCarFromSleep
*	功能说明: 设置机器标志位，设置是否是从睡眠中醒来的
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_setCleanCarFromSleep(bool val)
{
	isFormSleep = val;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_IsSelfCheckingReady
*	功能说明: 返回是否自检完毕
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_IsSelfCheckingReady(void)
{
	return isSelfCheckingReady;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetSelfCheckingReady
*	功能说明: 设置是否自检完毕
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetSelfCheckingReady(bool chk)
{
	isSelfCheckingReady = chk;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_StartPowerOnToggle
*	功能说明: 开启开机时刻的初始化状态灯
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartPowerOnToggle(void)
{
	powerOnToggle.action = 0 ;
	powerOnToggle.delay = 0 ;
	powerOnToggle.times = 0 ;
	powerOnToggle.isRunning = true;
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StopPowerOnToggle
*	功能说明: 关闭开机时刻的初始化状态灯
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopPowerOnToggle(void)
{
	powerOnToggle.isRunning = false;
	powerOnToggle.action = 0 ;
	powerOnToggle.delay = 0 ;
	powerOnToggle.times = 0 ;
	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_PowerOnToggle
*	功能说明: 开机时刻的初始化状态灯
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_PowerOnToggle(void)
{
	if(!powerOnToggle.isRunning)
		return;
	
	switch(powerOnToggle.action)
	{
		case 0: /*亮  POWER_ON_LED_TIME_INTERVAL*/
		{
			if(!isCleanCarFromSleep())
			{
				bsp_LedOn(LED_LOGO_CLEAN);
				bsp_LedOn(LED_LOGO_POWER);
				bsp_LedOn(LED_LOGO_CHARGE);

				powerOnToggle.delay = xTaskGetTickCount();
				powerOnToggle.action++;
			}
			else
			{
				bsp_LedOn(LED_LOGO_CLEAN);
				bsp_LedOn(LED_LOGO_POWER);
				bsp_LedOn(LED_LOGO_CHARGE);

				powerOnToggle.delay = xTaskGetTickCount();
				powerOnToggle.action = 3;
			}
		}break;
		
		case 1: /*灭  POWER_ON_LED_TIME_INTERVAL*/
		{
			if(xTaskGetTickCount() - powerOnToggle.delay >= POWER_ON_LED_TIME_INTERVAL)
			{
				bsp_LedOff(LED_LOGO_CLEAN);
				bsp_LedOff(LED_LOGO_POWER);
				bsp_LedOff(LED_LOGO_CHARGE);
				powerOnToggle.delay = xTaskGetTickCount();
				powerOnToggle.action++;
			}
		}break;
		
		case 2: /*灭  POWER_ON_LED_TIME_INTERVAL*/
		{
			if(xTaskGetTickCount() - powerOnToggle.delay >= POWER_ON_LED_TIME_INTERVAL)
			{
				powerOnToggle.action = 0;
				
				/*闪烁的次数到了*/
				if(++powerOnToggle.times >=  POWER_ON_LED_MAX_TIMES)
				{
					powerOnToggle.delay = xTaskGetTickCount();
					/*常亮10 min*/
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					
					bsp_SetSelfCheckingReady(true);
					
					powerOnToggle.action = 3;
				}
			}
		}break;
		
		case 3: /*10  分钟后熄灭LED，进入低功耗*/
		{
			if(xTaskGetTickCount() - powerOnToggle.delay >=  POWER_ON_LED_ON_CONSTANT_MAX_TIME)
			{
				bsp_LedOff(LED_LOGO_CLEAN);
				bsp_LedOff(LED_LOGO_POWER);
				bsp_LedOff(LED_LOGO_CHARGE);

				bsp_StopPowerOnToggle();
			}
			
		}break;
	}
	
	
}


typedef struct
{
	bool isRunning;
	uint32_t action;
	uint32_t delay;
	uint32_t times;
	
	LED_SN sn;
	
}RunToggleLED;

static RunToggleLED runToggleLED;

/*
*********************************************************************************************************
*	函 数 名: bsp_StartRunToggleLED
*	功能说明: 开启机器闪烁状态机，也可以是组合灯
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartRunToggleLED(LED_SN sn)
{
	runToggleLED.sn = sn;
	
	runToggleLED.action = 0 ;
	runToggleLED.delay = 0 ;
	runToggleLED.isRunning = true;

}


/*
*********************************************************************************************************
*	函 数 名: bsp_StopRunToggleLED
*	功能说明: 停止机器闪烁状态机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopRunToggleLED(void)
{
	runToggleLED.isRunning = false;
	runToggleLED.action = 0 ;
	runToggleLED.delay = 0 ;
	
	bsp_LedOn(LED_LOGO_CLEAN);
	bsp_LedOn(LED_LOGO_POWER);
	bsp_LedOn(LED_LOGO_CHARGE);
	bsp_LedOff(LED_COLOR_YELLOW);
	bsp_LedOff(LED_COLOR_GREEN);
	bsp_LedOff(LED_COLOR_RED);
	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_StopRunToggleLED
*	功能说明: 机器灯闪烁状态机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_RunToggleLED(void)
{
	if(!runToggleLED.isRunning)
		return;
	
	switch(runToggleLED.action)
	{
		case 0:
		{
			switch(runToggleLED.sn)
			{
				case LED_LOGO_CLEAN:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}break;
				
				case LED_LOGO_POWER:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}break;
				
				case LED_LOGO_CHARGE:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}break;
				
				case LED_COLOR_YELLOW:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOff(LED_LOGO_CHARGE);
					bsp_LedOn(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}break;
				
				case LED_COLOR_GREEN:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOff(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOn(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}break;
				
				case LED_COLOR_RED:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOff(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOn(LED_COLOR_RED);
				}break;
				
				case LED_WIFI_LINK:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
					
					runToggleLED.times = WIFI_SMART_CONFIG_TOGGLE_TIMES;
				}break;
			}
			
			runToggleLED.delay = xTaskGetTickCount();
			runToggleLED.action++;
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - runToggleLED.delay >= (runToggleLED.sn == LED_WIFI_LINK ? 300 : 500))
			{
				if(runToggleLED.sn == LED_WIFI_LINK) /*用于重置WIFI连接*/
				{
					if(runToggleLED.times >= 1)
					{
						bsp_LedToggle(LED_LOGO_CLEAN);
						bsp_LedToggle(LED_LOGO_CHARGE);
						--runToggleLED.times;
					}
					else
					{
						bsp_LedOn(LED_LOGO_CLEAN);
						bsp_LedOn(LED_LOGO_CHARGE);
					}
					
				}
				else
				{
					bsp_LedToggle(runToggleLED.sn);
				}
				
				
				runToggleLED.delay = xTaskGetTickCount();
				runToggleLED.action++;
			}
		}break;
		
		case 2:
		{
			if(xTaskGetTickCount() - runToggleLED.delay >= (runToggleLED.sn == LED_WIFI_LINK ? 300 : 500)) 
			{
				if(runToggleLED.sn == LED_WIFI_LINK) /*用于重置WIFI连接*/
				{
					
					if(runToggleLED.times >= 1)
					{
						bsp_LedToggle(LED_LOGO_CLEAN);
						bsp_LedToggle(LED_LOGO_CHARGE);
						--runToggleLED.times;
					}
					else
					{
						bsp_LedOn(LED_LOGO_CLEAN);
						bsp_LedOn(LED_LOGO_CHARGE);
					}
				}
				else
				{
					bsp_LedToggle(runToggleLED.sn);
				}
				
				runToggleLED.delay = xTaskGetTickCount();
				runToggleLED.action = 1;
			}
			
		}break;
	}
	
}

