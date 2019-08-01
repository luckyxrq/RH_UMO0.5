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


void bsp_InitDC_Motor(void);      /* 初始化直流电机的PWM定时器 */
void bsp_MotorBrake(MotorSN sn);  /* 急停 */
void bsp_MotorCoast(MotorSN sn);  /* 滑行停止 */
void bsp_SetMotorPWM(MotorSN sn, MotorDir dir, uint16_t pwm);  /* 直接使用占空比控制，6000基本是最低速度，再小就不转了 */

#endif

