#ifndef __BSP_DC_MOTOR_H
#define __BSP_DC_MOTOR_H


#define MOTOR_NUM    2       /*�������*/
#define MAXPWM       10000   /*���PWMֵ*/
#define DC_PWM_T     30000   /*DC�������оƬռ�ձ�����*/


typedef enum
{
	MotorLeft = 0 ,/*��ߵ�����*/
	MotorRight     /*�ұߵ�����*/ 
}MotorSN;

typedef enum
{
	Forward  = 0 ,/*ǰ��*/
	Backward      /*����*/ 
}MotorDir;


void bsp_InitDC_Motor(void);
void bsp_MotorBrake(MotorSN sn);
void bsp_MotorCoast(MotorSN sn);
void bsp_SetMotorPWM(MotorSN sn, MotorDir dir, uint16_t pwm);

#endif

