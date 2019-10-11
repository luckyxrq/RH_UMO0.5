#ifndef __BSP_RUN_CONTROL_H
#define __BSP_RUN_CONTROL_H

#include <stdbool.h>
#include "bsp_led.h"

/*ÔËÐÐ×´Ì¬*/
typedef enum
{
	RUN_STATE_DEFAULT = 0 ,
	RUN_STATE_CLEAN,
	RUN_STATE_CHARGE,
	RUN_STATE_SHUTDOWN
}RunState;


bool bsp_IsSelfCheckingReady(void);
void bsp_SetSelfCheckingReady(bool chk);

void bsp_StartPowerOnToggle(void);
void bsp_StopPowerOnToggle(void);
void bsp_PowerOnToggle(void);

void bsp_StartRunToggleLED(LED_SN sn);
void bsp_StopRunToggleLED(void);
void bsp_RunToggleLED(void);


#endif

