#ifndef __BSP_SEARCHCHARGEPILE_H
#define __BSP_SEARCHCHARGEPILE_H

bool bsp_IsCharging(void);
bool bsp_IsChargeDone(void);
void bsp_StartSearchChargePile(void);
void bsp_StopSearchChargePile(void);
void bsp_SearchChargePile(void);
bool bsp_IsTouchChargePile(void);
#endif

