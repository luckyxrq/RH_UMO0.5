#ifndef __BSP_RUN_CONTROL_H
#define __BSP_RUN_CONTROL_H

typedef enum
{
	LED_DEFAULT_STATE = 0,
	THREE_WHITE_TOOGLE,       /*三颗白色LED一起闪*/
	THREE_WHITE_ON,           /*三颗白色LED一起亮*/
	AT_CHARGING,
	AT_CHARGE_DONE,
	
	AT_SEARCH_CHARGE,
	AT_CLEAN,
	AT_LINK
}LedAppState;

typedef enum
{
	forward,
	backward,
	turn_left,
	turn_right,
	stop
}WORK_DIRECTION_CONTROL;

typedef enum
{
	idle,
	smart_clean,
	wall_clean,
	goto_charge,
	charging,
	charge_done,
	paused
}WORK_STATUS;

typedef enum
{
	smart,
	wall_follow,
	standby
}WORK_MODE;

extern uint8_t  work_status;
extern uint8_t  work_mode;
extern uint8_t  work_switch_go;

void bsp_CloseAllLed(void);
void bsp_OpenThreeWhileLed(void);
void bsp_LedAppProc(void);
void bsp_SetLedState(LedAppState ledAppState);
void bsp_PowerOnLedProc(void);
LedAppState bsp_GetLedAppState(void);

#endif

