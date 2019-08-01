#ifndef __BSP_POWERSWITCH_H
#define __BSP_POWERSWITCH_H


typedef enum
{
	SW_5V_EN_CTRL = 0 ,
	SW_IR_POWER
}SW_ID;


void bsp_SwOn(SW_ID sw);
void bsp_SwOff(SW_ID sw);
void bsp_InitSW(void);


#endif

