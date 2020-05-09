#ifndef __BSP_CLIFF_H
#define __BSP_CLIFF_H

#include <stdbool.h>

typedef enum
{
	CliffLeft = 0 ,
	CliffMiddle,
	CliffRight
}CliffSWSN;

void bsp_InitCliffSW(void);
float bsp_GetCliffVoltage(CliffSWSN sn);
void bsp_CliffCalibration(void);


bool bsp_CliffIsDangerous(CliffSWSN sn);

uint8_t bsp_GetCliffStates(void);

void bsp_CliffTest(void);

void bsp_StartCliffTest(void);
void bsp_StopCliffTest(void);

#endif


