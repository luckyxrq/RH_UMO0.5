#ifndef __BSP_CONTROL_H
#define __BSP_CONTROL_H

#include "bsp_motor.h"

void bsp_InitPid(MotorSN sn);
void bsp_PidSched(void);
void bsp_SetMotorSpeed(MotorSN sn , int32_t speed);
int32_t bsp_MotorGetSpeed(MotorSN sn);
int32_t bsp_MotorGetTargetSpeed(MotorSN sn);
int32_t bsp_MotorSpeedMM2Pulse(int16_t mm);
int32_t bsp_MotorGetPulseVector(MotorSN sn);
void bsp_PidClear(MotorSN sn);
void bsp_ClearMotorPulseVector(void);
#endif

