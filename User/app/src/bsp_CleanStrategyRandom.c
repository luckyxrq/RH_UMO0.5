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
			
			if( strategyRandom.collision != CollisionNone)
			
			{
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-120));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-120));
				
				/*��¼�´�ʱ��������*/
				strategyRandom.pulse = bsp_GetCurrentBothPulse();
				
				++strategyRandom.action;
			}
			else /*û����ײ��ֱ���ߣ�������*/
			{
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(120));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(120));
			}
			
		}break;
		
		case 1: /*��ײ�˾�����*/
		{
			if(bsp_GetCurrentBothPulse() - strategyRandom.pulse >= GO_BACK_PULSE)
			{
				strategyRandom.angle = bsp_AngleRead();
				strategyRandom.delay = xTaskGetTickCount();
				
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+120));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-120));
				
				++strategyRandom.action;
			}
		}break;
		
		case 2: /*���˾�ԭ��ת*/
		{
			if((xTaskGetTickCount() - strategyRandom.delay)>= 500)
			{
				strategyRandom.action = 0 ;
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(120));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(120));
			}
		}break;
		
	}
}





