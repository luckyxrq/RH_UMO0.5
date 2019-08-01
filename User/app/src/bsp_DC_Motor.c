#include "bsp.h"





typedef struct
{
	bool isRunning;     /*电机是否在运行*/
	uint32_t encodeCnt; /*编码器计数值*/
	
}Motor;


/*电机管理结构体数组*/
Motor motor[MOTOR_NUM];

static void bspInitPWM(void);


/*
*********************************************************************************************************
*	函 数 名: bsp_InitDC_Motor
*	功能说明: 初始化DC电机。
*			 全局变量。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitDC_Motor(void)
{
	bspInitPWM();
}

/*
*********************************************************************************************************
*	函 数 名: bspInitPWM
*	功能说明: 初始化DC电机需要的定时器，PWM输出。
*			 全局变量。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bspInitPWM(void)
{
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);           //Timer1重映射     
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,0, MAXPWM);     //当频率为0，占空比为100%时，GPIO输出1
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,0, MAXPWM);     //当频率为0，占空比为100%时，GPIO输出1
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3,0, MAXPWM);     //当频率为0，占空比为100%时，GPIO输出1
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,0, MAXPWM);     //当频率为0，占空比为100%时，GPIO输出1
}

/*
*********************************************************************************************************
*	函 数 名: bsp_MotorBrake
*	功能说明: 电机急停
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_MotorBrake(MotorSN sn)
{
	switch(sn)
	{
		case MotorLeft:
		{
			motor[MotorLeft].isRunning = false ;
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3,0,MAXPWM); 
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,0,MAXPWM);
		}break;
		
		case MotorRight:
		{
			motor[MotorRight].isRunning = false ;
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,0,MAXPWM);
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,0,MAXPWM);
		}break;
	}
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_MotorCoast
*	功能说明: 电机滑行停
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_MotorCoast(MotorSN sn)
{
	switch(sn)
	{
		case MotorLeft:
		{
			motor[MotorLeft].isRunning = false ;
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3,0,0); 
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,0,0);
		}break;
		
		case MotorRight:
		{
			motor[MotorRight].isRunning = false ;
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,0,0);
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,0,0);
		}break;
	}
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetMotorPWM
*	功能说明: 电机编号，方向，占空比（有限制，限制在头文件）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetMotorPWM(MotorSN sn, MotorDir dir, uint16_t pwm)
{	
	//如果给出的PWM已经大于最大值，则直接退出函数
	if(pwm > MAXPWM)
	{
		WARNING("PWM值超过了MAXPWM\r\n");
		return ;
	}
	
	switch(sn)
	{
		case MotorLeft:
		{
			motor[MotorLeft].isRunning = true ;
			if(dir == Forward)
			{
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3,0, 0); 
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,DC_PWM_T, pwm);
			}
			else
			{
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,0, 0);
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3, DC_PWM_T,pwm); 
			}
		}break;
		
		case MotorRight:
		{
			motor[MotorRight].isRunning = true ;	
			if(dir == Forward)
			{
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,0, 0);
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,DC_PWM_T, pwm);
			}
			else
			{
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,0, 0);
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,DC_PWM_T, pwm);
			}
		}break;
	}
}






