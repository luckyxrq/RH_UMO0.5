#ifndef __BSP_CLIFF_H
#define __BSP_CLIFF_H


typedef enum
{
	Cliff1_left = 0 ,
	Cliff2_middle,
	Cliff3_right
}CliffSWSN;

void bsp_InitCliffSW(void);
float bsp_GetCliffVoltage(CliffSWSN sn);

#endif


