#ifndef __BSP_DC_MOTOR_H
#define __BSP_DC_MOTOR_H


#define MOTOR_NUM    2       /*电机数量*/
#define MAXPWM       10000   /*最大PWM值*/
#define DC_PWM_T     30000   /*DC电机驱动芯片占空比周期*/


typedef enum
{
	MotorLeft = 0 ,/*左边电机编号*/
	MotorRight     /*右边电机编号*/ 
}MotorSN;

typedef enum
{
	Forward  = 0 ,/*前进*/
	Backward      /*后退*/ 
}MotorDir;


void bsp_InitDC_Motor(void);
void bsp_MotorBrake(MotorSN sn);
void bsp_MotorCoast(MotorSN sn);
void bsp_SetMotorPWM(MotorSN sn, MotorDir dir, uint16_t pwm);

#endif

