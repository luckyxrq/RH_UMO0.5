#include "bsp.h"




/*
*********************************************************************************************************
*	函 数 名: bsp_InitPinPulse
*	功能说明: 初始化脉冲引脚，用于示波器协助检测
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitPinPulse(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_ALL_PULSE, ENABLE );	 
                      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_PULSE;
	GPIO_Init(GPIO_PORT_PULSE, &GPIO_InitStructure);	
}

typedef struct
{
	bool isRunning;
	uint8_t action;
	uint32_t delay;
	
	float lowPulseV;    /*这里的高低针对的是单片机，示波器是反过来的*/
	float highPulseV;   /*这里的高低针对的是单片机，示波器是反过来的*/
	bool  isDangerM;    /*中间跳崖是否危险*/
}CliffPulse;

static CliffPulse cliffPulse;

#define M_CLIFF_THRESHOLD      0.06F /*中间跳崖阈值*/


bool bsp_GetIsDangerCliff_M(void)
{
	return cliffPulse.isDangerM;
}


void bsp_CliffPulseDetect(void)
{
//	if(!cliffPulse.isRunning)
//		return;
	
	switch(cliffPulse.action)
	{
		case 0:
		{
			M_CLIFF_PULSE_LOW();
			cliffPulse.delay = xTaskGetTickCount();
			++cliffPulse.action;
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - cliffPulse.delay >= 1)
			{
				cliffPulse.lowPulseV = bsp_GetCliffVoltage(CliffMiddle);
				//DEBUG("cliffPulse.lowPulseV:%.2f\r\n",cliffPulse.lowPulseV);
				
				M_CLIFF_PULSE_HIGH();
				
				bsp_DelayUS(200);
				
				cliffPulse.highPulseV = bsp_GetCliffVoltage(CliffMiddle);
				M_CLIFF_PULSE_LOW();
				//DEBUG("cliffPulse.highPulseV:%.2f\r\n",cliffPulse.highPulseV);
				//DEBUG("sub:%.2F\r\n",cliffPulse.lowPulseV-cliffPulse.highPulseV);
				if(cliffPulse.lowPulseV-cliffPulse.highPulseV <= M_CLIFF_THRESHOLD)
				{
					cliffPulse.isDangerM = true;
				}
				else
				{
					cliffPulse.isDangerM = false;
				}
				
				DEBUG("%.2F  %.2F  %.2F  %s\r\n",
				cliffPulse.lowPulseV,
				cliffPulse.highPulseV,
				cliffPulse.lowPulseV-cliffPulse.highPulseV,
				cliffPulse.isDangerM?"√√√√√√":"XXXXXX");
				
				cliffPulse.action = 0;
			}
		}break;
		
//		case 2:
//		{
//			if(xTaskGetTickCount() - cliffPulse.delay >= 1)
//			{
//				cliffPulse.highPulseV = bsp_GetCliffVoltage(CliffMiddle);
//				//DEBUG("cliffPulse.highPulseV:%.2f\r\n",cliffPulse.highPulseV);
//				//DEBUG("sub:%.2F\r\n",cliffPulse.lowPulseV-cliffPulse.highPulseV);
//				if(cliffPulse.lowPulseV-cliffPulse.highPulseV <= M_CLIFF_THRESHOLD)
//				{
//					cliffPulse.isDangerM = true;
//				}
//				else
//				{
//					cliffPulse.isDangerM = false;
//				}
//				
//				DEBUG("%.2F  %s\r\n",
//				cliffPulse.lowPulseV-cliffPulse.highPulseV,
//				cliffPulse.isDangerM?"悬崖真":"悬崖假");
//				
//				cliffPulse.action = 0;
//			}
//		}break;
	}
}

bool isStartCliffTest = false;

void bsp_StartCliffPulseTest(void)
{
	isStartCliffTest = true;
}

void bsp_CliffPulseTest(void)
{
	if(isStartCliffTest)
	{
		if(!bsp_GetIsDangerCliff_M())
		{
			bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(200));
			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(200));
		}		
		else
		{
			isStartCliffTest = false;
			bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(0));
			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
		}
	}
	
}
