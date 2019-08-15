#ifndef __BSP_CONTROL_H
#define __BSP_CONTROL_H

#include "bsp_motor.h"

int32_t myabs(int32_t val);
void bsp_InitPid(MotorSN sn);
void bsp_PidClear(MotorSN sn);
void bsp_PidExec(MotorSN sn , int32_t Encoder, int32_t Target);

#endif

