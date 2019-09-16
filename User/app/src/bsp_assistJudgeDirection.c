#include "bsp.h"

#define HISTORICAL_RECORD_COUNT      20
#define THRESHOLD_OBSTACLE           1.0F

typedef struct
{
	bool isRunning;
	uint32_t delay;
	uint32_t action;
	
	bool historyRemote[HISTORICAL_RECORD_COUNT];
	uint32_t historyIndex;
}AssistJudgeDirection;

static AssistJudgeDirection assistJudgeDirection;

/*
*********************************************************************************************************
*	�� �� ��: bsp_StartAssistJudgeDirection
*	����˵��: ����Э���ж��Ƿ�ǰ����ײ��
*	��    ��: ��
*	�� �� ֵ: ��
*   �� �� ��: 4  
*********************************************************************************************************
*/
void bsp_StartAssistJudgeDirection(void)
{
	assistJudgeDirection.action = 0 ;
	assistJudgeDirection.delay = 0 ;
	assistJudgeDirection.isRunning = 0 ;
	assistJudgeDirection.historyIndex = 0 ;
	memset(assistJudgeDirection.historyRemote,0,HISTORICAL_RECORD_COUNT);
	
	assistJudgeDirection.isRunning = true;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StopAssistJudgeDirection
*	����˵��: ֹͣЭ���ж��Ƿ�ǰ����ײ��
*	��    ��: ��
*	�� �� ֵ: ��
*   �� �� ��: 4  
*********************************************************************************************************
*/
void bsp_StopAssistJudgeDirection(void)
{
	assistJudgeDirection.isRunning = false;
	
	assistJudgeDirection.action = 0 ;
	assistJudgeDirection.delay = 0 ;
	assistJudgeDirection.isRunning = 0 ;
	assistJudgeDirection.historyIndex = 0 ;
	memset(assistJudgeDirection.historyRemote,0,HISTORICAL_RECORD_COUNT);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_AssistJudgeDirection
*	����˵��: Э���ж��Ƿ���ǰ����ײ��
*	��    ��: ��
*	�� �� ֵ: ��
*   �� �� ��: 4  
*********************************************************************************************************
*/
void bsp_AssistJudgeDirection(void)
{
	if(!assistJudgeDirection.isRunning)
		return;
	
	/*ǰ������״̬*/
	if(bsp_GetInfraRedAdcVoltage(IR3) >= THRESHOLD_OBSTACLE)
	{
		assistJudgeDirection.historyRemote[assistJudgeDirection.historyIndex] = true;
	}
	else
	{
		assistJudgeDirection.historyRemote[assistJudgeDirection.historyIndex] = false;
	}
	
	
	/*��ż���*/
	assistJudgeDirection.historyIndex++;
	if(assistJudgeDirection.historyIndex >= HISTORICAL_RECORD_COUNT)
	{
		assistJudgeDirection.historyIndex = 0 ;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_AssistIsFaceObstacles
*	����˵��: Э���ж��Ƿ���ǰ����ײ��
*	��    ��: ��
*	�� �� ֵ: ��
*   �� �� ��: 4  
*********************************************************************************************************
*/
bool bsp_AssistIsFaceObstacles(void)
{
	uint8_t trueCount = 0 ;
	uint8_t i = 0 ;
	bool ret;
	
	for(i=0;i<HISTORICAL_RECORD_COUNT;i++)
	{
		if(assistJudgeDirection.historyRemote[i] == true)
		{
			trueCount++;
		}
	}
	
	if(trueCount >= 2)
	{
		ret = true;
	}
	else
	{
		ret = false;
	}
	
	return ret;
	
}

