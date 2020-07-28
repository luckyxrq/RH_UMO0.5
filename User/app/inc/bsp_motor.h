#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H


#include <stdbool.h>


#define CONSTANT_HIGH_PWM      (3600)
#define CONSTANT_LOW_PWM       (0)


#define VACUUM_DEFAULT_PER      60//80

typedef enum
{
	MotorLeft = 0 ,
	MotorRight
}MotorSN;

typedef enum
{
	MotorRollingBrush = 0 ,
	MotorSideBrush
}MotorCleanSN;

typedef enum
{
	Forward = 0 ,
	Backward
}MotorDir;

typedef enum
{
	CW = 0 ,
	CCW
}MotorCleanDir;

typedef struct
{
	volatile bool isRunning;
	volatile uint32_t tick;
}Vacuum;

void bsp_InitMotor(void);
void bsp_MotorSetPWM(MotorSN sn ,MotorDir dir,uint16_t pwm);
void bsp_MotorCleanSetPWM(MotorCleanSN sn, MotorCleanDir dir , uint16_t pwm);

void bsp_StartVacuum(uint8_t grade);
void bsp_StopVacuum(void);

#endif

