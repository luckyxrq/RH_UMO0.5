#include "bsp.h"

#define POWER_ON_LED_TIME_INTERVAL            500
#define POWER_ON_LED_MAX_TIMES                5
#define POWER_ON_LED_ON_CONSTANT_MAX_TIME     10*60*1000         

typedef struct
{
	/*上电闪烁10S，500MS闪烁一次，共20次*/
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
	
	/*闪烁次数计数*/
	volatile uint8_t times;
}PowerOnToggle;

static PowerOnToggle powerOnToggle;
static RunControl runControl;
static bool isSelfCheckingReady = false;


/*
*********************************************************************************************************
*	函 数 名: bsp_IsSelfCheckingReady
*	功能说明: 返回是否自检完毕
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_IsSelfCheckingReady(void)
{
	return isSelfCheckingReady;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetSelfCheckingReady
*	功能说明: 设置是否自检完毕
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetSelfCheckingReady(bool chk)
{
	isSelfCheckingReady = chk;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_StartPowerOnToggle
*	功能说明: 开启开机时刻的初始化状态灯
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartPowerOnToggle(void)
{
	powerOnToggle.action = 0 ;
	powerOnToggle.delay = 0 ;
	powerOnToggle.times = 0 ;
	powerOnToggle.isRunning = true;
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StopPowerOnToggle
*	功能说明: 关闭开机时刻的初始化状态灯
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopPowerOnToggle(void)
{
	powerOnToggle.isRunning = false;
	powerOnToggle.action = 0 ;
	powerOnToggle.delay = 0 ;
	powerOnToggle.times = 0 ;
	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_PowerOnToggle
*	功能说明: 开机时刻的初始化状态灯
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_PowerOnToggle(void)
{
	if(!powerOnToggle.isRunning)
		return;
	
	switch(powerOnToggle.action)
	{
		case 0: /*亮  POWER_ON_LED_TIME_INTERVAL*/
		{
			bsp_LedOn(LED_LOGO_CLEAN);
			bsp_LedOn(LED_LOGO_POWER);
			bsp_LedOn(LED_LOGO_CHARGE);
			powerOnToggle.delay = xTaskGetTickCount();
			powerOnToggle.action++;
		}break;
		
		case 1: /*灭  POWER_ON_LED_TIME_INTERVAL*/
		{
			if(xTaskGetTickCount() - powerOnToggle.delay >= POWER_ON_LED_TIME_INTERVAL)
			{
				bsp_LedOff(LED_LOGO_CLEAN);
				bsp_LedOff(LED_LOGO_POWER);
				bsp_LedOff(LED_LOGO_CHARGE);
				powerOnToggle.delay = xTaskGetTickCount();
				powerOnToggle.action++;
			}
		}break;
		
		case 2: /*灭  POWER_ON_LED_TIME_INTERVAL*/
		{
			if(xTaskGetTickCount() - powerOnToggle.delay >= POWER_ON_LED_TIME_INTERVAL)
			{
				powerOnToggle.action = 0;
				
				/*闪烁的次数到了*/
				if(++powerOnToggle.times >=  POWER_ON_LED_MAX_TIMES)
				{
					powerOnToggle.delay = xTaskGetTickCount();
					/*常亮10 min*/
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					
					bsp_SetSelfCheckingReady(true);
					
					powerOnToggle.action = 3;
				}
			}
		}break;
		
		case 3: /*10  分钟后熄灭LED，进入低功耗*/
		{
			if(xTaskGetTickCount() - powerOnToggle.delay >=  POWER_ON_LED_ON_CONSTANT_MAX_TIME)
			{
				bsp_LedOff(LED_LOGO_CLEAN);
				bsp_LedOff(LED_LOGO_POWER);
				bsp_LedOff(LED_LOGO_CHARGE);

				bsp_StopPowerOnToggle();
			}
			
		}break;
	}
	
	
}


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
		
		//DEBUG("isHomeKey\r\n");
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
		
		//DEBUG("isChargeKey\r\n");
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
		
		//DEBUG("isCleanKey\r\n");
	}
	/*Power按键被按下*/
    else if(runControl.isPowerKey)
    {
		runControl.isPowerKey = false;
		
		DEBUG("关机\r\n");
		
		bsp_InitRunControl();
		bsp_StartRunControl();
		
		bsp_SwOff(SW_5V_EN_CTRL);
		bsp_SwOff(SW_IR_POWER);
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
				
				DEBUG("初始化机器...\r\n");
			}
			/*扫地工作*/
			else if(runControl.currentState == ROBOT_STATE_WORKING)
			{
				DEBUG("开始清扫...\r\n");
			}
			/*暂停*/
			else if(runControl.currentState == ROBOT_STATE_SUSPEND)
			{
				DEBUG("暂停...\r\n");
			}
			/*回充*/
			else if(runControl.currentState == ROBOT_STATE_CHARGING)
			{
				DEBUG("回充...\r\n");
			}
			/*待机*/
			else if(runControl.currentState == ROBOT_STATE_STANDBY)
			{
				DEBUG("待机...\r\n");
			}
		}break;
		
	}
}



