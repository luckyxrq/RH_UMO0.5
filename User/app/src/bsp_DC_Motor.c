#include "bsp.h"


typedef struct
{
	volatile bool isRunning;     /*电机是否在运行*/
	volatile uint32_t encodeCnt; /*编码器计数值*/
	
}Motor;


/*电机管理结构体数组*/
Motor motor[MOTOR_NUM];
static PID pid[2];

static void bspInitPWM(void);
static float pidabs(float val);

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
			if(dir == Backward)
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


/*
*********************************************************************************************************
*	函 数 名: bsp_InitMotorPid
*	功能说明: 初始化某个电机PID
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitMotorPid(MotorSN sn)
{
	switch(sn)
	{
		case MotorLeft:
		{
			/************************PID 左轮机**********12 0.5 1 0.8 5500*******/
			pid[MotorLeft].target = 150;
			
			pid[MotorLeft].kp = 6;
			pid[MotorLeft].ki = 0.5;
			pid[MotorLeft].kd = 1;
			
			pid[MotorLeft].bias = 0 ;
			pid[MotorLeft].lastBias = 0 ;
			pid[MotorLeft].biasSum = 0 ;
			
			pid[MotorLeft].pwm = 0 ;
			pid[MotorLeft].pwmMax = MAXPWM ;
			
			pid[MotorLeft].kiLimit = 20000 ;
			pid[MotorLeft].fitK = 0.8 ;
			pid[MotorLeft].fitD = 2000 ;
		}break;
		
		case MotorRight:
		{
			/************************PID 右轮机************************/
			pid[MotorRight].target = 150;
			
			pid[MotorRight].kp = 6;
			pid[MotorRight].ki = 0.5;
			pid[MotorRight].kd = 1;
			
			pid[MotorRight].bias = 0 ;
			pid[MotorRight].lastBias = 0 ;
			pid[MotorRight].biasSum = 0 ;
			
			pid[MotorRight].pwm = 0 ;
			pid[MotorRight].pwmMax = MAXPWM ;
			
			pid[MotorRight].kiLimit = 20000 ;
			pid[MotorRight].fitK = 0.8 ;
			pid[MotorRight].fitD = 2000 ;
		}break;
	}
}



/*
*********************************************************************************************************
*	函 数 名: bsp_PidControlAct
*	功能说明: 实时调节电机PID，100MS调用一次
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_PidControlAct(void)
{
	int32_t speed = 0 ;
	float S_Out[4] = {0};
	
	UNUSED(S_Out);
	
	/************************显示速度波形************************/
	#if 0 
	S_Out[0] = bsp_EncoderGetSpeed(EncoderLeft);
	S_Out[1] = bsp_EncoderGetSpeed(EncoderRight);
	SDS_OutPut_Data(S_Out);
	#endif
	
	/************************PID  左轮************************/
	if(motor[MotorLeft].isRunning)
	{
		speed = bsp_EncoderGetSpeed(EncoderLeft);//当前速度毫米/秒
		pid[MotorLeft].bias = pidabs(pid[MotorLeft].target) - speed;//当前误差值
		pid[MotorLeft].biasSum += pid[MotorLeft].bias;//误差累积
		
		//积分限制
		if(pid[MotorLeft].biasSum >= pid[MotorLeft].kiLimit)
		{
			pid[MotorLeft].biasSum = pid[MotorLeft].kiLimit;
		}
			
		pid[MotorLeft].pwm = (pid[MotorLeft].fitK*pidabs(pid[MotorLeft].target) + pid[MotorLeft].fitD) + pid[MotorLeft].kp*pid[MotorLeft].bias + pid[MotorLeft].ki*pid[MotorLeft].biasSum + pid[MotorLeft].kd*(pid[MotorLeft].bias-pid[MotorLeft].lastBias); 
		pid[MotorLeft].lastBias = pid[MotorLeft].bias;//上次误差
		
		//PWM限制
		if(pid[MotorLeft].pwm >= pid[MotorLeft].pwmMax)
		{
			pid[MotorLeft].pwm = pid[MotorLeft].pwmMax;
		}
		else if(pid[MotorLeft].pwm <= 0)
		{
			pid[MotorLeft].pwm = 0 ;
		}
		#if 0
		printf("left:%dmm/s  ",bsp_EncoderGetSpeed(EncoderLeft));
		printf("pid[MotorLeft].bias:%f  ",pid[MotorLeft].bias);
		printf("pid[MotorLeft].biasSum:%f  ",pid[MotorLeft].biasSum);
		printf("pid[MotorLeft].lastBias:%f  ",pid[MotorLeft].lastBias);
		printf("pid[MotorLeft].pwm:%f  ",pid[MotorLeft].pwm);
		#endif
		bsp_SetMotorPWM(MotorLeft, pid[MotorLeft].target>0 ? Forward:Backward, pid[MotorLeft].pwm);
	}
	
	
	/************************PID  右轮************************/
	if(motor[MotorRight].isRunning)
	{
		speed = bsp_EncoderGetSpeed(EncoderRight);//当前速度毫米/秒
		pid[MotorRight].bias = pidabs(pid[MotorRight].target) - speed;//当前误差值
		pid[MotorRight].biasSum += pid[MotorRight].bias;//误差累积
		
		//积分限制
		if(pid[MotorRight].biasSum >= pid[MotorRight].kiLimit)
		{
			pid[MotorRight].biasSum = pid[MotorRight].kiLimit;
		}
			
		pid[MotorRight].pwm = (pid[MotorRight].fitK*pidabs(pid[MotorRight].target) + pid[MotorRight].fitD) + pid[MotorRight].kp*pid[MotorRight].bias + pid[MotorRight].ki*pid[MotorRight].biasSum + pid[MotorRight].kd*(pid[MotorRight].bias-pid[MotorRight].lastBias); 
		pid[MotorRight].lastBias = pid[MotorRight].bias;//上次误差
		
		//PWM限制
		if(pid[MotorRight].pwm >= pid[MotorRight].pwmMax)
		{
			pid[MotorRight].pwm = pid[MotorRight].pwmMax;
		}
		else if(pid[MotorRight].pwm <= 0)
		{
			pid[MotorRight].pwm = 0 ;
		}
		#if 0
		printf("right:%dmm/s  ",bsp_EncoderGetSpeed(EncoderRight));
		printf("pid[MotorRight].bias:%f  ",pid[MotorRight].bias);
		printf("pid[MotorRight].biasSum:%f  ",pid[MotorRight].biasSum);
		printf("pid[MotorRight].lastBias:%f  ",pid[MotorRight].lastBias);
		printf("pid[MotorRight].pwm:%f\r\n",pid[MotorRight].pwm);
		#endif
		bsp_SetMotorPWM(MotorRight, pid[MotorRight].target>0 ? Forward:Backward, pid[MotorRight].pwm);
	}
	
}

static float pidabs(float val)
{
	return val>0 ? val : -val;
}





