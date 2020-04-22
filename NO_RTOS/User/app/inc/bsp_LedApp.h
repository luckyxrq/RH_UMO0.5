#ifndef __BSP_LEDAPP_H
#define __BSP_LEDAPP_H

typedef enum
{
	LED_DEFAULT_STATE = 0,
	THREE_WHITE_TOOGLE,       /*三颗白色LED一起闪*/
	THREE_WHITE_ON,           /*三颗白色LED一起亮*/
	AT_CHARGING,
	AT_CHARGE_DONE,
	
}LedAppState;

void bsp_CloseAllLed(void);
void bsp_OpenThreeWhileLed(void);
void bsp_LedAppProc(void);
void bsp_SetLedState(LedAppState ledAppState);

#endif

