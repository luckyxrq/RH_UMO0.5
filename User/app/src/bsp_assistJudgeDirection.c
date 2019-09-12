#include "bsp.h"

#define HISTORICAL_RECORD_COUNT      10
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
	if(bsp_GetInfraRedAdcVoltage(IR2) >= THRESHOLD_OBSTACLE || bsp_GetInfraRedAdcVoltage(IR3) >= THRESHOLD_OBSTACLE || bsp_GetInfraRedAdcVoltage(IR4) >= THRESHOLD_OBSTACLE)
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

