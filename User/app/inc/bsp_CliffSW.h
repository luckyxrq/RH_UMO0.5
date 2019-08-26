#ifndef __BSP_CLIFF_H
#define __BSP_CLIFF_H


typedef enum
{
	Cliff1 = 0 ,
	Cliff2,
	Cliff3
}CliffSWSN;

void bsp_InitCliffSW(void);
float bsp_GetCliffVoltage(CliffSWSN sn);

#endif


