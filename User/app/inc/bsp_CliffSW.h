#ifndef __BSP_CLIFF_H
#define __BSP_CLIFF_H


typedef enum
{
	CliffLeft = 0 ,
	CliffMiddle,
	CliffRight
}CliffSWSN;

void bsp_InitCliffSW(void);
float bsp_GetCliffVoltage(CliffSWSN sn);

#endif


