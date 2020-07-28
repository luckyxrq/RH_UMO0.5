#ifndef __BSP_PARAM_H
#define __BSP_PARAM_H


/*******************************************风机 风力***************************************************/
#define VACUUM_STRENGTH  0 /*强力*/
#define VACUUM_NORMAL    1 /*正常*/
#define VACUUM_QUIET     2 /*安静*/



void bsp_LoadParam(void);
void bsp_SaveParam(void);
void bsp_ParamReadAtPowerOn(void);


void bsp_SetVacuumPowerGrade(uint8_t grade);
uint8_t bsp_GetVacuumPowerGrade(void);

#endif

