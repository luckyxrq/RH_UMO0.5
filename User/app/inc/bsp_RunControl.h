#ifndef __BSP_RUN_CONTROL_H
#define __BSP_RUN_CONTROL_H

#include <stdbool.h>

typedef struct
{
	/*״̬���ṹ*/
	volatile bool isRunnng;
	volatile uint32_t action;
	volatile uint32_t delay;
	
	/*����״̬*/
	volatile RunState lastState;
	volatile RunState currentState;
	volatile WorkMethod workMethod;
	volatile uint8_t errSN;
	
	/*����ֵ��Home������������Power�ػ�*/
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

