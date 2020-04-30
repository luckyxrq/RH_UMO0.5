#ifndef __BSP_RUN_CONTROL_H
#define __BSP_RUN_CONTROL_H

typedef enum
{
	LED_DEFAULT_STATE = 0,
	THREE_WHITE_TOOGLE,       /*���Ű�ɫLEDһ����*/
	THREE_WHITE_ON,           /*���Ű�ɫLEDһ����*/
	AT_CHARGING,
	AT_CHARGE_DONE,
	
	AT_SEARCH_CHARGE,
	AT_CLEAN,
	AT_LINK
}LedAppState;

void bsp_CloseAllLed(void);
void bsp_OpenThreeWhileLed(void);
void bsp_LedAppProc(void);
void bsp_SetLedState(LedAppState ledAppState);
void bsp_PowerOnLedProc(void);
LedAppState bsp_GetLedAppState(void);

#endif

