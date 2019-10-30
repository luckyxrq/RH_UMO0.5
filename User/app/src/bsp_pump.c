#include "bsp.h"

#define PUMP_OPEN_MS       2000   /*水泵开时间*/
#define PUMP_CLOSE_MS      4000   /*水泵关时间*/

/*水泵控制*/
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
*	函 数 名: bsp_InitPump
*	功能说明: 初始化水泵
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitPump(void)
{
	/*
		水泵的控制口，使用的是扫地机的边刷控制口，
		边刷控制口默认是高电平，但是高电平水泵就动了，
		所以初始化的时候就得把水泵关掉
	*/
	bsp_PumpClose();
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StartPumpRun
*	功能说明: 开启水泵运行循环
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: bsp_StopPumpRun
*	功能说明: 关闭水泵运行循环
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: bsp_PumpRun
*	功能说明: 水泵运行循环
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_PumpRun(void)
{
	if(!pump.isRunning)
		return;
	
	switch(pump.action)
	{
		case 0: /*开水泵*/
		{
			bsp_PumpOpen();
			pump.delay = xTaskGetTickCount();
			pump.action++;
		}break;
		
		case 1: /*开足时间 PUMP_OPEN_MS ，然后关水泵*/
		{
			if(xTaskGetTickCount() - pump.delay >= PUMP_OPEN_MS)
			{
				bsp_PumpClose();
				pump.delay = xTaskGetTickCount();
				pump.action++;
			}
		}break;
		
		case 2: /*关足时间 bsp_PumpClose ，然后回到循环的开始*/
		{
			if(xTaskGetTickCount() - pump.delay >= PUMP_CLOSE_MS)
			{
				pump.action = 0 ;
			}
		}break;
	}
}
