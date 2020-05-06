#include "bsp.h"

/*轮子后退20MM，脉冲数*/               
#define GO_BACK_PULSE                  (10/(3.14F*70)*1024)

typedef struct
{
	bool isRunning;
	uint8_t action;
	uint32_t delay;
	
	bool leftCliff;
	bool middleCliff;
	bool rightCliff;
	float angle;
	Collision collision;
	uint32_t pulse;
	uint32_t runStartTick;
}StrategyRandom;

static StrategyRandom strategyRandom;

void bsp_StartStrategyRandom(void)
{
	strategyRandom.action = 0 ;
	strategyRandom.delay = 0 ;
	strategyRandom.runStartTick = xTaskGetTickCount() ;
	strategyRandom.isRunning = true;
	
}


void bsp_StopStrategyRandom(void)
{
	strategyRandom.isRunning = false;
	strategyRandom.action = 0 ;
	strategyRandom.delay = 0 ;
	strategyRandom.runStartTick = 0 ;
}


void bsp_StrategyRandomProc(void)
{
	if(!strategyRandom.isRunning)
		return;
	
	
	if(xTaskGetTickCount() - strategyRandom.runStartTick >= 1*60*60*1000) /*运行超过1小时*/
	{
		if(xTaskGetTickCount() - strategyRandom.runStartTick <= (1*60*60*1000 + 15*60*1000)) /*休息15分钟*/
		{
			bsp_SetMotorSpeed(MotorLeft, 0);
			bsp_SetMotorSpeed(MotorRight,0);
			bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
			bsp_MotorCleanSetPWM(MotorRollingBrush, CW , 0);
			bsp_StopVacuum();
			
			return; /*休息15分钟的时候直接返回*/
		}
		else /*15分钟后重新开设备，并且重新算开始运行的时刻*/
		{
			bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
			bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
			bsp_StartVacuum();
			
			strategyRandom.runStartTick = xTaskGetTickCount() ;
		}
	}
	
	
	
	switch(strategyRandom.action)
	{
		case 0: /*开机直接先跑*/
		{
			strategyRandom.collision = bsp_CollisionScan();
			
			strategyRandom.leftCliff = bsp_CliffIsDangerous(CliffLeft);
			strategyRandom.middleCliff = bsp_CliffIsDangerous(CliffMiddle);
			strategyRandom.rightCliff = bsp_CliffIsDangerous(CliffRight);
			
			if( strategyRandom.collision != CollisionNone || 
				strategyRandom.leftCliff ||
			    strategyRandom.middleCliff ||
			    strategyRandom.rightCliff) /*有碰撞或者悬崖就后退*/
			{
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-200));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-200));
				
				/*记录下此时的脉冲数*/
				strategyRandom.pulse = bsp_GetCurrentBothPulse();
				
				++strategyRandom.action;
			}
			else /*没有碰撞就直接走，快速走*/
			{
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(250));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
			}
			
		}break;
		
		case 1: /*碰撞了就先退*/
		{
			if(bsp_GetCurrentBothPulse() - strategyRandom.pulse >= GO_BACK_PULSE)
			{
				strategyRandom.angle = bsp_AngleRead();
				strategyRandom.delay = xTaskGetTickCount();
				
				if(strategyRandom.collision == CollisionLeft || strategyRandom.leftCliff)
				{
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				}
				else if(strategyRandom.collision == CollisionRight || strategyRandom.rightCliff)
				{
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+100));
				}
				else if(strategyRandom.collision == CollisionAll || strategyRandom.middleCliff)
				{
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				}
				
				++strategyRandom.action;
			}
		}break;
		
		case 2: /*退了就原地转*/
		{
			if((xTaskGetTickCount() - strategyRandom.delay)>= 2000 || 
				abs(bsp_AngleAdd(strategyRandom.angle ,36) - (bsp_AngleRead())) <= 10.0F)
			{
				strategyRandom.action = 0 ;
			}
		}break;
		
	}
}





