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

static void bsp_RunControl(void)
{
	if(!runControl.isRunnng)
		return;
	
	switch(runControl.action)
	{
		case 0:
		{
			
		}break;
	}
}
