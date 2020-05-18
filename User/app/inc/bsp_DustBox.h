#ifndef __DUSTBOX_H
#define __DUSTBOX_H

typedef enum
{
	DustBoxInside = 0,
	DustBoxOutside
}DustBoxState;

void bsp_InitDustBox(void);
DustBoxState bsp_DustBoxGetState(void);

void bsp_StartDustBoxProc(void);
void bsp_StopDustBoxProc(void);
void bsp_DustBoxProc(void);


#endif

