#ifndef __BSP_RUN_CONTROL_H
#define __BSP_RUN_CONTROL_H

#include <stdbool.h>

typedef struct
{
	/*״̬���ṹ*/
	bool isRunnng;
	uint32_t action;
	uint32_t delay;
	
	/*����״̬*/
	RunState lastState;
	RunState currentState;
	WorkMethod workMethod;
	uint8_t errSN;
	
	/*����ֵ��Home������������Power�ػ�*/
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

