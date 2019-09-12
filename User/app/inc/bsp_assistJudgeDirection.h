#ifndef __BSP_ASSISTJUDGEDIRECTION_H
#define __BSP_ASSISTJUDGEDIRECTION_H

#include <stdbool.h>

void bsp_StartAssistJudgeDirection(void);
void bsp_StopAssistJudgeDirection(void);
void bsp_AssistJudgeDirection(void);
bool bsp_AssistIsFaceObstacles(void);

#endif
