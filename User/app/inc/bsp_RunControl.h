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

#endif

