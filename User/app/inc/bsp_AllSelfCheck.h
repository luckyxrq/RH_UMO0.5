#ifndef __BSP_ALLSELFCHECK_H
#define __BSP_ALLSELFCHECK_H


void bsp_StartAllSelfCheck(void);
void bsp_StopAllSelfCheck(void);
bool bsp_IsRunningAllSelfCheck(void);
void bsp_AllSelfCheckProc(void);

void bsp_SetIMU_OK(void);
void bsp_SetWIFI_OK(void);
void bsp_AllSelfCheckSendFrame(uint16_t tx , uint16_t rx , uint16_t main , uint16_t sub , uint8_t data[] , uint16_t size);

#endif



