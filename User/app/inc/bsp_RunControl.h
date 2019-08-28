#ifndef __BSP_RUN_CONTROL_H
#define __BSP_RUN_CONTROL_H

#include <stdbool.h>

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

void bsp_InitRunControl(void);
void bsp_RunControl(void);
void bsp_SetHomeKey(bool val);
void bsp_SetPowerKey(bool val);
void bsp_SetChargeKey(bool val);
void bsp_SetCleanKey(bool val);
void bsp_StartRunControl(void);
void bsp_StopRunControl(void);

#endif

