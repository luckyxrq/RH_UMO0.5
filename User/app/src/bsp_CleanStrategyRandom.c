#include "bsp.h"

/*���Ӻ���20MM��������*/               
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
	
	uint32_t randomStartTick;
}StrategyRandom;

static StrategyRandom strategyRandom;

void bsp_StartStrategyRandom(void)
{
	strategyRandom.action = 0 ;
	strategyRandom.delay = 0 ;
	strategyRandom.randomStartTick = xTaskGetTickCount(); /*��¼����ʱ��*/
	strategyRandom.angle = bsp_AngleRead(); /*��¼�����Ƕ�*/
	strategyRandom.isRunning = true;
	
}


void bsp_StopStrategyRandom(void)
{
	strategyRandom.isRunning = false;
	strategyRandom.action = 0 ;
	strategyRandom.delay = 0 ;
	
}

bool bsp_IsStartStrategyRandom(void)
{
	return strategyRandom.isRunning;
}


void bsp_StrategyRandomProc(void)
{
	if(!strategyRandom.isRunning)
		return;
	
	
	switch(strategyRandom.action)
	{
		case 0:
		{
			bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+150));
			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-150));
			
			if(  (xTaskGetTickCount() - strategyRandom.randomStartTick >= 2000 && abs(strategyRandom.angle - bsp_AngleRead()) <= 10)  ||  bsp_CollisionScan() != CollisionNone)
			{
				strategyRandom.action++;
			}
		}break;
		
		case 1: /*����ֱ������*/
		{
			strategyRandom.collision = bsp_CollisionScan();
			
			strategyRandom.leftCliff = bsp_CliffIsDangerous(CliffLeft);
			strategyRandom.middleCliff = bsp_CliffIsDangerous(CliffMiddle);
			strategyRandom.rightCliff = bsp_CliffIsDangerous(CliffRight);
			
			if( strategyRandom.collision != CollisionNone || 
				strategyRandom.leftCliff ||
			    strategyRandom.middleCliff ||
			    strategyRandom.rightCliff) /*����ײ�������¾ͺ���*/
			{
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-200));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-200));
				
				/*��¼�´�ʱ��������*/
				strategyRandom.pulse = bsp_GetCurrentBothPulse();
				
				++strategyRandom.action;
			}
			else /*û����ײ��ֱ���ߣ�������*/
			{
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(250));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
			}
			
		}break;
		
		case 2: /*��ײ�˾�����*/
		{
			if(bsp_GetCurrentBothPulse() - strategyRandom.pulse >= GO_BACK_PULSE)
			{
				strategyRandom.angle = bsp_AngleRead();
				strategyRandom.delay = xTaskGetTickCount();
				
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+250));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-250));
				
//				if(strategyRandom.collision == CollisionLeft || strategyRandom.leftCliff)
//				{
//					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+250));
//					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-250));
//				}
//				else if(strategyRandom.collision == CollisionRight || strategyRandom.rightCliff)
//				{
//					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-100));
//					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+100));
//				}
//				else if(strategyRandom.collision == CollisionAll || strategyRandom.middleCliff)
//				{
//					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+100));
//					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
//				}
				
				++strategyRandom.action;
			}
		}break;
		
		case 3: /*���˾�ԭ��ת*/
		{
			if((xTaskGetTickCount() - strategyRandom.delay)>= 800 || 
				abs(bsp_AngleAdd(strategyRandom.angle ,36) - (bsp_AngleRead())) <= 10.0F)
			{
				strategyRandom.action = 0 ;
			}
		}break;
		
	}
}





