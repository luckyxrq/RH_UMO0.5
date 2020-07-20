#ifndef __BSP_EDGEWISE_H
#define __BSP_EDGEWISE_H

typedef enum 
{
	Dir_left = 0,
	Dir_right = 1,
	Dir_default = 2
}Dir_Right_Left;

void bsp_StartEdgewiseRun(void);
void bsp_StopEdgewiseRun(void);
void bsp_EdgewiseRun(void);
void bsp_SetEdgeLeftRight(Dir_Right_Left Edg_dir);

#endif

