#include "bsp.h"

typedef struct
{
	volatile bool isRunning;
	volatile uint8_t action;
	volatile uint32_t delay;
}RunStableAct;


typedef struct
{
	volatile bool isRunning;
	volatile uint8_t action;
	volatile uint32_t delay;
	volatile float lastAngle;
}TurnAngleAct;


static RunStableAct runStableAct;
static TurnAngleAct turnAngleAct;

/*
*********************************************************************************************************
*	函 数 名: bsp_StartRunStable
*	功能说明: 开启平缓启动。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartRunStable(void)
{
	runStableAct.action = 0 ;
	runStableAct.isRunning = true;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StopRunStable
*	功能说明: 停止平缓启动。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopRunStable(void)
{
	runStableAct.isRunning = false;
	runStableAct.action = 0 ;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_RunStableAct
*	功能说明: 平缓启动。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_RunStableAct(void)
{
	if(!runStableAct.isRunning)
	{
		return ;
	}
	
	switch(runStableAct.action)
	{
		case 0:
		{
			bsp_SetMotorPWM(MotorLeft,Forward,6000);
			bsp_SetMotorPWM(MotorRight,Forward,6000);
			runStableAct.delay = xTaskGetTickCount();
			runStableAct.action++;
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - runStableAct.delay >= 1000)
			{
				bsp_SetMotorPWM(MotorLeft,Forward,7000);
				bsp_SetMotorPWM(MotorRight,Forward,7000);
				runStableAct.delay = xTaskGetTickCount();
				runStableAct.action++;
			}
		}break;
		
		case 2:
		{
			if(xTaskGetTickCount() - runStableAct.delay >= 1000)
			{
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				
				runStableAct.isRunning = false;
				runStableAct.action = 0 ;
			}
		}break;
	}
}



void bsp_TurnAngleAct(int16_t angle)
{
	if(!turnAngleAct.isRunning)
	{
		return;
	}
	
	switch(turnAngleAct.action)
	{
		case 0:
		{
			
		}break;
		
		case 1:
		{
			
		}break;
	}
	
}



