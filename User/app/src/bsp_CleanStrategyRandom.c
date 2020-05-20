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
}StrategyRandom;

static StrategyRandom strategyRandom;

void bsp_StartStrategyRandom(void)
{
	strategyRandom.action = 0 ;
	strategyRandom.delay = 0 ;
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


static uint32_t CIRCLE_DELAY[6] = {500,800,1000,1200,1500,2000};
static uint32_t CIRCLE_DELAY_INDEX = 0 ;

void bsp_StrategyRandomProc(void)
{
	
	uint8_t random_num = 0;
	
	if(!strategyRandom.isRunning)
		return;
	
	
	switch(strategyRandom.action)
	{
		case 0: /*����ֱ������*/
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
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-250));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-250));
				
				/*��¼�´�ʱ��������*/
				strategyRandom.pulse = bsp_GetCurrentBothPulse();
				
				++strategyRandom.action;
			}
			else /*û����ײ��ֱ���ߣ�������*/
			{
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(220));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(220));
			}
			
		}break;
		
		case 1: /*��ײ�˾�����*/
		{
			if(bsp_GetCurrentBothPulse() - strategyRandom.pulse >= GO_BACK_PULSE)
			{
				strategyRandom.angle = bsp_AngleRead();
				strategyRandom.delay = xTaskGetTickCount();
				
				if(xTaskGetTickCount()%2 == 0 )
				{
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-190));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+190));
				}
				else
				{
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+190));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-190));
				}
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
		
		case 2: /*���˾�ԭ��ת*/
		{
			if((xTaskGetTickCount() - strategyRandom.delay)>= CIRCLE_DELAY[++CIRCLE_DELAY_INDEX %6])
			{
				strategyRandom.action = 0 ;
			}
		}break;
		
	}
}





