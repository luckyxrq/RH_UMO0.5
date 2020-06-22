#include "bsp.h"

#define FREE_STATE      (!bsp_IsTouchChargePile() &&  bsp_GetLedAppState() != AT_CLEAN && bsp_GetLedAppState() != AT_SEARCH_CHARGE &&bsp_GetLedAppState() != THREE_WHITE_TOOGLE &&bsp_GetLedAppState() != AT_LINK)
#define FREE_LONG_TIME_SLEEP     (5*60*1000) /*����5���� ���Ҳ���׮�����������ģʽ*/


typedef struct
{
	bool isRunning;
	uint32_t delay;
	uint32_t action;
	
	uint32_t lastFreeTime;
}SleepProc;

static SleepProc sleepProc;


void bsp_StartSleepProc(void)
{
	sleepProc.action = 0 ;
	sleepProc.delay = 0 ;

	sleepProc.lastFreeTime = 0;
	
	sleepProc.isRunning = true;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_SleepProc
*	����˵��: ��ʱ��û����ɨ���߻س䣬�Զ���������ģʽ
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SleepProc(void)
{
	if(!sleepProc.isRunning)
	{
		return;
	}
	
	switch(sleepProc.action)
	{
		case 0:
		{
			if(FREE_STATE)
			{
				sleepProc.lastFreeTime = xTaskGetTickCount();
				++sleepProc.action;
			}
		}break;
		
		case 1:
		{
			if(!FREE_STATE)
			{
				sleepProc.action = 0 ;
			}
			else
			{
				if(xTaskGetTickCount() - sleepProc.lastFreeTime >= FREE_LONG_TIME_SLEEP)
				{
					bsp_PutKey(KEY_LONG_POWER); /*ģ�ⰴ�����½�������ģʽ*/
					++sleepProc.action;
				}
			}
		}break;
		
		default:break;
	}
	
	
	
}

