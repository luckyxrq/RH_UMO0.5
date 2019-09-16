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
*	函 数 名: bsp_StartAssistJudgeDirection
*	功能说明: 开启协助判断是否前面碰撞。
*	形    参: 无
*	返 回 值: 无
*   优 先 级: 4  
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
*	函 数 名: bsp_StopAssistJudgeDirection
*	功能说明: 停止协助判断是否前面碰撞。
*	形    参: 无
*	返 回 值: 无
*   优 先 级: 4  
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
*	函 数 名: bsp_AssistJudgeDirection
*	功能说明: 协助判断是否是前面碰撞。
*	形    参: 无
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
void bsp_AssistJudgeDirection(void)
{
	if(!assistJudgeDirection.isRunning)
		return;
	
	/*前面红外的状态*/
	if(bsp_GetInfraRedAdcVoltage(IR3) >= THRESHOLD_OBSTACLE)
	{
		assistJudgeDirection.historyRemote[assistJudgeDirection.historyIndex] = true;
	}
	else
	{
		assistJudgeDirection.historyRemote[assistJudgeDirection.historyIndex] = false;
	}
	
	
	/*序号计数*/
	assistJudgeDirection.historyIndex++;
	if(assistJudgeDirection.historyIndex >= HISTORICAL_RECORD_COUNT)
	{
		assistJudgeDirection.historyIndex = 0 ;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_AssistIsFaceObstacles
*	功能说明: 协助判断是否是前面碰撞。
*	形    参: 无
*	返 回 值: 无
*   优 先 级: 4  
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

