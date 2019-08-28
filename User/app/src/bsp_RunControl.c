#include "bsp.h"



typedef struct
{
	/*状态机结构*/
	bool isRunnng;
	uint32_t action;
	uint32_t delay;
	
	/*运行状态*/
	RunState lastState;
	RunState currentState;
	WorkMethod workMethod;
	uint8_t errSN;
	
	/*按键值，Home按键长按就是Power关机*/
	bool isHomeKey;
	bool isPowerKey;
	bool isChargeKey;
	bool isCleanKey;
}RunControl;


static RunControl runControl;


/*
*********************************************************************************************************
*	函 数 名: bsp_InitRunControl
*	功能说明: 初始化运行状态
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitRunControl(void)
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
static void bsp_RunControl(void)
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
