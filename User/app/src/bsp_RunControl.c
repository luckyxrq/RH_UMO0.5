#include "bsp.h"


static RunControl runControl;


/*
*********************************************************************************************************
*	函 数 名: bsp_InitRunControl
*	功能说明: 初始化运行状态
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitRunControl(void)
{
	/*状态机结构*/
	runControl.isRunnng = false;
	runControl.action = 0 ;
	runControl.delay = 0 ;
	
	/*运行状态初始化*/
	runControl.lastState = ROBOT_STATE_DEFAULT;
	runControl.currentState = ROBOT_STATE_DEFAULT;
	runControl.workMethod = ROBOT_WORKWAY_DEFAULT;
	runControl.errSN = ROBOT_ERROR_NUM_DEFAULT;
	
	/*按键值初始化*/
	runControl.isHomeKey = false;
	runControl.isPowerKey = false;
	runControl.isChargeKey = false;
	runControl.isCleanKey = false;
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetHomeKey
*	功能说明: Home按键短按
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetHomeKey(bool val)
{
	runControl.isHomeKey = val;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetPowerKey
*	功能说明: Home按键长按，断电
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetPowerKey(bool val)
{
	runControl.isPowerKey = val;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetChargeKey
*	功能说明: 充电按键短按
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetChargeKey(bool val)
{
	runControl.isChargeKey = val;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetCleanKey
*	功能说明: 清扫按键短按
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetCleanKey(bool val)
{
	runControl.isCleanKey = val;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_StartRunControl
*	功能说明: 开启按键控制状态机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartRunControl(void)
{
	/*状态机结构*/
	runControl.action = 0 ;
	runControl.delay = 0 ;
	
	/*运行状态初始化*/
	runControl.lastState = ROBOT_STATE_DEFAULT;
	runControl.currentState = ROBOT_STATE_DEFAULT;
	runControl.workMethod = ROBOT_WORKWAY_DEFAULT;
	runControl.errSN = ROBOT_ERROR_NUM_DEFAULT;
	
	/*按键值初始化*/
	runControl.isHomeKey = false;
	runControl.isPowerKey = false;
	runControl.isChargeKey = false;
	runControl.isCleanKey = false;
	
	runControl.isRunnng = true;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_StopRunControl
*	功能说明: 关闭按键控制状态机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopRunControl(void)
{
	/*状态机结构*/
	runControl.isRunnng = false;
	runControl.action = 0 ;
	runControl.delay = 0 ;
	
	/*运行状态初始化*/
	runControl.lastState = ROBOT_STATE_DEFAULT;
	runControl.currentState = ROBOT_STATE_DEFAULT;
	runControl.workMethod = ROBOT_WORKWAY_DEFAULT;
	runControl.errSN = ROBOT_ERROR_NUM_DEFAULT;
	
	/*按键值初始化*/
	runControl.isHomeKey = false;
	runControl.isPowerKey = false;
	runControl.isChargeKey = false;
	runControl.isCleanKey = false;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_RunControl
*	功能说明: 机器运行状态切换调度，周期性被调用
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_RunControl(void)
{
	if(!runControl.isRunnng)
		return;
	
	/*更新上一次状态*/
	runControl.lastState = runControl.currentState;
    	
	/*HOME按键被按下*/
    if(runControl.isHomeKey)
    {
		runControl.isHomeKey = false;
		
        if(runControl.lastState == ROBOT_STATE_DEFAULT)       runControl.currentState = ROBOT_STATE_INIT;
        else if(runControl.lastState == ROBOT_STATE_WORKING)  runControl.currentState = ROBOT_STATE_SUSPEND;
        else if(runControl.lastState == ROBOT_STATE_CHARGING) runControl.currentState = ROBOT_STATE_SUSPEND;
        else if(runControl.lastState == ROBOT_STATE_SUSPEND)  runControl.currentState = ROBOT_STATE_WORKING;
        else if(runControl.lastState == ROBOT_STATE_STANDBY)  runControl.currentState = ROBOT_STATE_WORKING;
        
        runControl.workMethod  = ROBOT_WORKWAY_HOME;
		
		DEBUG("isHomeKey\r\n");
    }
	/*Charge按键被按下*/
    else if(runControl.isChargeKey)
    {
		runControl.isChargeKey = false;
		
        if(runControl.lastState == ROBOT_STATE_DEFAULT)       runControl.currentState = ROBOT_STATE_INIT;
        else if(runControl.lastState == ROBOT_STATE_WORKING)  runControl.currentState = ROBOT_STATE_CHARGING;
        else if(runControl.lastState == ROBOT_STATE_CHARGING) runControl.currentState = ROBOT_STATE_SUSPEND;
        else if(runControl.lastState == ROBOT_STATE_SUSPEND)  runControl.currentState = ROBOT_STATE_CHARGING;
        else if(runControl.lastState == ROBOT_STATE_STANDBY)  runControl.currentState = ROBOT_STATE_CHARGING;
        
        runControl.workMethod  = ROBOT_WORKWAY_CHARGE;
		
		DEBUG("isChargeKey\r\n");
    }
	/*Clean按键被按下*/
    else if(runControl.isCleanKey)
    {
		runControl.isCleanKey = false;
		
        if(runControl.lastState == ROBOT_STATE_DEFAULT)       runControl.currentState = ROBOT_STATE_INIT;
        else if(runControl.lastState == ROBOT_STATE_WORKING)  runControl.currentState = ROBOT_STATE_SUSPEND;
        else if(runControl.lastState == ROBOT_STATE_CHARGING) runControl.currentState = ROBOT_STATE_SUSPEND;
        else if(runControl.lastState == ROBOT_STATE_SUSPEND)  runControl.currentState = ROBOT_STATE_WORKING;
        else if(runControl.lastState == ROBOT_STATE_STANDBY)  runControl.currentState = ROBOT_STATE_WORKING;
        
        runControl.workMethod  = ROBOT_WORKWAY_CLEAN;
		
		DEBUG("isCleanKey\r\n");
    }
	
	switch(runControl.action)
	{
		case 0:/*判断当前状态，如果状态不为ROBOT_STATE_DEFAULT，则往下执行*/
		{
			if(runControl.currentState != ROBOT_STATE_DEFAULT)
			{
				runControl.action++;
			}
		}break;
		
		case 1:/*各个状态秩序井不同的操作*/
		{
			/*初始化硬件*/
			if(runControl.currentState == ROBOT_STATE_INIT)
			{
				
				/*初始化完毕后，根据清扫方式，进行不同的操作*/
				runControl.currentState = (runControl.workMethod == ROBOT_WORKWAY_CHARGE)? ROBOT_STATE_CHARGING:ROBOT_STATE_WORKING;
			}
			/*扫地工作*/
			else if(runControl.currentState == ROBOT_STATE_WORKING)
			{
				
			}
			/*暂停*/
			else if(runControl.currentState == ROBOT_STATE_SUSPEND)
			{
				
			}
			/*回充*/
			else if(runControl.currentState == ROBOT_STATE_CHARGING)
			{
				
			}
			/*待机*/
			else if(runControl.currentState == ROBOT_STATE_STANDBY)
			{
				
			}
		}break;
		
		
		
		
	}
}
