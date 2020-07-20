#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H


#include <stdbool.h>


#define CONSTANT_HIGH_PWM      (3600)
#define CONSTANT_LOW_PWM       (0)


#define VACUUM_DEFAULT_PER      80



#define VACUUM_0_AS_ELECTROLYTIC_H2O   TIM_SetCompare1(TIM2,CONSTANT_HIGH_PWM * 0.0F) /*�������IO�������ˮ��IO,��ʱ��PWM������ͨIO*/
#define VACUUM_1_AS_ELECTROLYTIC_H2O   TIM_SetCompare1(TIM2,CONSTANT_HIGH_PWM)        /*�������IO�������ˮ��IO,��ʱ��PWM������ͨIO*/


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

void bsp_StartVacuum(uint8_t pwm);
void bsp_StopVacuum(void);

#endif

