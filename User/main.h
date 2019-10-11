#ifndef __BSP_MAIN_H
#define __BSP_MAIN_H

#include "bsp_RunControl.h"

typedef struct
{
	volatile RunState lastRunState;
	volatile uint32_t lastKeyTick;
}KeyProc;

void bsp_SetKeyRunLastState(RunState state);
RunState bsp_GetKeyRunLastState(void);
void bsp_SetLastKeyTick(uint32_t tick);
uint32_t bsp_GetLastKeyTick(void);

#endif

