#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#define CONSTANT_HIGH_PWM      (7200)
#define CONSTANT_LOW_PWM       (0)


typedef enum
{
	MotorLeft = 0 ,
	MotorRight
}MotorSN;

typedef enum
{
	Forward = 0 ,
	Backward
}MotorDir;

void bsp_InitMotor(void);
void bsp_MotorSetPWM(MotorSN sn ,MotorDir dir,uint16_t pwm);

#endif

