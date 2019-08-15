#ifndef __BSP_CONTROL_H
#define __BSP_CONTROL_H

#include "bsp_motor.h"

int32_t myabs(int32_t val);
int32_t Incremental_PI (MotorSN sn ,int32_t Encoder,int32_t Target);
int32_t Position_PID (int32_t Encoder,int32_t Target);

#endif

