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
	
	
	if(xTaskGetTickCount() - strategyRandom.runStartTick >= 1*60*60*1000) /*���г���1Сʱ*/
	{
		if(xTaskGetTickCount() - strategyRandom.runStartTick <= (1*60*60*1000 + 15*60*1000)) /*��Ϣ15����*/
		{
			bsp_SetMotorSpeed(MotorLeft, 0);
			bsp_SetMotorSpeed(MotorRight,0);
			bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
			bsp_MotorCleanSetPWM(MotorRollingBrush, CW , 0);
			bsp_StopVacuum();
			
			return; /*��Ϣ15���ӵ�ʱ��ֱ�ӷ���*/
		}
		else /*15���Ӻ����¿��豸�����������㿪ʼ���е�ʱ��*/
		{
			bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
			bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
			bsp_StartVacuum();
			
			strategyRandom.runStartTick = xTaskGetTickCount() ;
		}
	}
	
	
	
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
		
		case 1: /*��ײ�˾�����*/
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
		
		case 2: /*���˾�ԭ��ת*/
		{
			if((xTaskGetTickCount() - strategyRandom.delay)>= 2000 || 
				abs(bsp_AngleAdd(strategyRandom.angle ,36) - (bsp_AngleRead())) <= 10.0F)
			{
				strategyRandom.action = 0 ;
			}
		}break;
		
	}
}





