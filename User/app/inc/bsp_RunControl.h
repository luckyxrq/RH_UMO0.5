#ifndef __BSP_RUN_CONTROL_H
#define __BSP_RUN_CONTROL_H

#include <stdbool.h>

typedef struct
{
	/*状态机结构*/
	volatile bool isRunnng;
	volatile uint32_t action;
	volatile uint32_t delay;
	
	/*运行状态*/
	volatile RunState lastState;
	volatile RunState currentState;
	volatile WorkMethod workMethod;
	volatile uint8_t errSN;
	
	/*按键值，Home按键长按就是Power关机*/
	volatile bool isHomeKey;
	volatile bool isPowerKey;
	volatile bool isChargeKey;
	volatile bool isCleanKey;

}RunControl;



void bsp_StartPowerOnToggle(void);
void bsp_StopPowerOnToggle(void);
void bsp_PowerOnToggle(void);

void bsp_InitRunControl(void);
void bsp_RunControl(void);
void bsp_SetHomeKey(bool val);
void bsp_SetPowerKey(bool val);
void bsp_SetChargeKey(bool val);
void bsp_SetCleanKey(bool val);
void bsp_StartRunControl(void);
void bsp_StopRunControl(void);

#endif

