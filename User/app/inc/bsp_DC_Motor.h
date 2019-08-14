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

typedef struct
{
	volatile float target; //目标值
	
	volatile float kp;  //比例
	volatile float ki;  //积分
	volatile float kd;  //微分
	
	volatile float bias;     //本次误差
	volatile float lastBias; //上次误差
	volatile float biasSum;  //累积误差
	
	volatile float pwm;     //计算出的PWM值
	volatile float pwmMax;  //PWM能够取的最大值
	
	volatile float kiLimit; //积分上限
	volatile float fitK;    //拟合参数k  
	volatile float fitD;    //拟合参数d
}PID;

void bsp_InitDC_Motor(void);      /* 初始化直流电机的PWM定时器 */
void bsp_MotorBrake(MotorSN sn);  /* 急停 */
void bsp_MotorCoast(MotorSN sn);  /* 滑行停止 */
void bsp_SetMotorPWM(MotorSN sn, MotorDir dir, uint16_t pwm);  /* 直接使用占空比控制，6000基本是最低速度，再小就不转了 */
void bsp_PidControlAct(void);
void bsp_SetMotorTargetSpeed(MotorSN sn, float targetSpeed);
MotorDir bsp_MotorGetDir(MotorSN sn);
void bsp_MotorRunR(void);
void bsp_MotorRunL(void);
void bsp_MotorRun(void);
#endif

