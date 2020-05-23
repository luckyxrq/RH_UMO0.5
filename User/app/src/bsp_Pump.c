#include "bsp.h"

extern PARAM_T param;


//#define PUMP_ON()       bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM)
//#define PUMP_OFF()      bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0)


#define PUMP_ON()       bsp_MotorCleanSetPWM(MotorSideBrush, CW , 2500)
#define PUMP_OFF()      bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0)


typedef struct
{
	bool isRunning;
	uint32_t action;
	uint32_t delay;
}Pump;

static Pump pump;

void bsp_StartPump(void)
{
	PUMP_OFF();
	pump.action = 0 ;
	pump.delay = 0 ;
	pump.isRunning = true;
	
}

void bsp_StopPump(void)
{
	pump.isRunning = false;
	pump.action = 0 ;
	pump.delay = 0 ;
	PUMP_OFF();
	
}


void bsp_PumpProc(void)
{
	if(!pump.isRunning)
		return;
	
	switch(pump.action)
	{
		case 0:
		{
			PUMP_ON();
			pump.delay = xTaskGetTickCount();
			++pump.action;
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - pump.delay >= 1000)
			{
				PUMP_OFF();
				pump.delay = xTaskGetTickCount();
				++pump.action;
			}
		}break;
		
		case 2:
		{
			if(xTaskGetTickCount() - pump.delay >= 1000*3)
			{
				PUMP_ON();
				pump.delay = xTaskGetTickCount();
				pump.action = 0;
			}
		}break;
	}
	
}

