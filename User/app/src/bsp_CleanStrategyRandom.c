#include "bsp.h"

/*���Ӻ���20MM��������*/               
#define GO_BACK_PULSE                  (30/(3.14F*70)*1024)

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
	
	bsp_StopEdgewiseRun();
	
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
	static unsigned char IRSensorData[10] = {0};
	
	UNUSED(random_num);
	
	if(!strategyRandom.isRunning)
		return;
	
	
	bsp_GetAllIrIsObstacle(IRSensorData);
	
	switch(strategyRandom.action)
	{
		case 0: /*����ֱ������*/
		{
			strategyRandom.collision = bsp_CollisionScan();
			
			strategyRandom.leftCliff = bsp_CliffIsDangerous(CliffLeft);
			strategyRandom.middleCliff = bsp_CliffIsDangerous(CliffMiddle);
			strategyRandom.rightCliff = bsp_CliffIsDangerous(CliffRight); 
			
			if( strategyRandom.collision == CollisionAll || 
				strategyRandom.leftCliff ||
			    strategyRandom.middleCliff ||
			    strategyRandom.rightCliff) /*��ȫ��ײ�������¾ͺ���*/
			{
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-120));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-120));
				
				/*��¼�´�ʱ��������*/
				strategyRandom.pulse = bsp_GetCurrentBothPulse();
				
				strategyRandom.action =  1;
			}
			else if(strategyRandom.collision == CollisionRight)
			{
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-120));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-120));
				
				/*��¼�´�ʱ��������*/
				strategyRandom.pulse = bsp_GetCurrentBothPulse();
				strategyRandom.action =  3;
			}
			else if(strategyRandom.collision == CollisionLeft)
			{
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-120));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-120));
				
				/*��¼�´�ʱ��������*/
				strategyRandom.pulse = bsp_GetCurrentBothPulse();
				strategyRandom.action =  5;
			}
			else /*û����ײ��ֱ���ߣ�������*/
			{
				if(IRSensorData[2] == 1 || IRSensorData[7] == 1 || IRSensorData[0] == 1 ||\
				   IRSensorData[4] == 1 || IRSensorData[6] == 1)
				{
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(120));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(120));			
				}
				else
				{
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(180));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(180));
				}
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
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+100));
				}
				else
				{
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				}

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
		
		
		case 3: /*��ײ�˾�����*/
		{
			if(bsp_GetCurrentBothPulse() - strategyRandom.pulse >= GO_BACK_PULSE)
			{
				strategyRandom.angle = bsp_AngleRead();
				strategyRandom.delay = xTaskGetTickCount();
				bsp_StartEdgewiseRun();
				bsp_SetEdgeLeftRight(Dir_right);
				strategyRandom.action = 4;
			}
		}break;
		case 4: /*���˾�right�ر�*/
		{
			if((xTaskGetTickCount() - strategyRandom.delay)>= 3000 && 
					ABS(strategyRandom.angle - bsp_AngleRead()) <= 50.0F)
			{
				bsp_StopEdgewiseRun();
				strategyRandom.action = 0 ;
			}
		}break;
		
		
		case 5: /*��ײ�˾�����*/
		{
			if(bsp_GetCurrentBothPulse() - strategyRandom.pulse >= GO_BACK_PULSE)
			{
				strategyRandom.angle = bsp_AngleRead();
				strategyRandom.delay = xTaskGetTickCount();
				bsp_StartEdgewiseRun();
				bsp_SetEdgeLeftRight(Dir_left);
				strategyRandom.action = 6;
			}
		}break;
		case 6: /*���˾�left�ر�*/
		{
			if((xTaskGetTickCount() - strategyRandom.delay)>= 3000 && 
					ABS(strategyRandom.angle - bsp_AngleRead()) <= 50.0F)
			{
				bsp_StopEdgewiseRun();
				strategyRandom.action = 0 ;
			}
		}break;
		
		
		
		
		
		
	}
}





