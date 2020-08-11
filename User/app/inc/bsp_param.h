#ifndef __BSP_PARAM_H
#define __BSP_PARAM_H

#define PARAM_VER			 20200811             /* �����汾 */
#define PARAM_SAVE_PAGE      255                  /* ���������ҳ��� */


/*******************************************��� ����***************************************************/
#define VACUUM_STRENGTH  0 /*ǿ��*/
#define VACUUM_NORMAL    1 /*����*/
#define VACUUM_QUIET     2 /*����*/



void bsp_LoadParam(void);
void bsp_SaveParam(void);
void bsp_ParamReadAtPowerOn(void);


void bsp_SetVacuumPowerGrade(uint8_t grade);
void bsp_SetParaCliff_L(uint32_t val);
void bsp_SetParaCliff_M(uint32_t val);
void bsp_SetParaCliff_R(uint32_t val);
void bsp_SetParaEdge_L(uint32_t val);
void bsp_SetParaEdge_R(uint32_t val);
void bsp_SetParaErLangShen(uint32_t val);


uint8_t bsp_GetVacuumPowerGrade(void);
uint32_t bsp_GetParaCliff_L(void);
uint32_t bsp_GetParaCliff_M(void);
uint32_t bsp_GetParaCliff_R(void);
uint32_t bsp_GetParaEdge_L(void);
uint32_t bsp_GetParaEdge_R(void);
uint32_t bsp_GetParaErLangShen(void);




#endif

