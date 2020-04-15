#include "bsp.h"


/*
	笔记：
	硬件前提：bsp_InitTimer1(7199,0);10KHz
	输出恒为0：TIM_SetCompare1(TIM1,0);
	输出恒为1：TIM_SetCompare1(TIM1,7200);!!!千万别写成TIM_SetCompare1(TIM1,7199);;
*/

/* 防碰撞触动开关对应的RCC时钟 */
#define RCC_ALL_VACUUM 	(RCC_APB2Periph_GPIOA)

#define GPIO_PORT_VACUUM   GPIOA
#define GPIO_PIN_VACUUM	   GPIO_Pin_0




static Vacuum vacuum;
static void bsp_InitTimer1(uint16_t arr,uint16_t psc);
static void bsp_InitTimer4(uint16_t arr,uint16_t psc);
static void bsp_InitVacuum(void);

/*
*********************************************************************************************************
*	函 数 名: bsp_InitMotor
*	功能说明: 初始化电机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitMotor(void)
{
	/*初始化PWM 20KHZ，用于驱动电机，10KHZ会有噪声*/
	bsp_InitTimer1(3599,0); 
	bsp_InitTimer4(3599,0);
	bsp_InitVacuum();
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_MotorSetPWM
*	功能说明: 通过PWM直接控制电机，PWM较小的时候，电机无法转动
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_MotorSetPWM(MotorSN sn ,MotorDir dir,uint16_t pwm)
{
	switch(sn)
	{
		case MotorLeft:
		{
			if(dir == Forward)
			{
				TIM_SetCompare3(TIM1,0);
				TIM_SetCompare4(TIM1,pwm);
			}
			else
			{
				TIM_SetCompare3(TIM1,pwm);
				TIM_SetCompare4(TIM1,0);
			}
		}break;
		
		case MotorRight:
		{
			if(dir == Forward)
			{
				TIM_SetCompare1(TIM1,0);
				TIM_SetCompare2(TIM1,pwm);
			}
			else
			{
				TIM_SetCompare1(TIM1,pwm);
				TIM_SetCompare2(TIM1,0);
			}
		}break;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_MotorCleanSetPWM
*	功能说明: 通过PWM直接控制电机，PWM较小的时候，电机无法转动，专门用于清扫的电机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_MotorCleanSetPWM(MotorCleanSN sn, MotorCleanDir dir , uint16_t pwm)
{
	switch(sn)
	{
		case MotorRollingBrush:
		{
			if(dir == CW)
			{
				TIM_SetCompare1(TIM4,0);
				TIM_SetCompare2(TIM4,pwm);
			}
			else
			{
				TIM_SetCompare1(TIM4,pwm);
				TIM_SetCompare2(TIM4,0);
			}
		}break;
		
		case MotorSideBrush:
		{
			if(dir == CW)
			{
				TIM_SetCompare3(TIM4,0);
				TIM_SetCompare4(TIM4,pwm);
			}
			else
			{
				TIM_SetCompare3(TIM4,pwm);
				TIM_SetCompare4(TIM4,0);
			}
		}break;

	}
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitVacuum
*	功能说明: 初始化吸尘器IO口，使用IO口模拟PWM
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitVacuum(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_VACUUM, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_VACUUM;
	GPIO_Init(GPIO_PORT_VACUUM, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIO_PORT_VACUUM,GPIO_PIN_VACUUM);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_VacuumClean
*	功能说明: 周期性调用，用于吸尘器
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_VacuumClean(void)
{	
	const uint32_t max_tick = 10;
	const uint32_t changeTick = 9;
	
	if(!vacuum.isRunning)
		return ;
	
	
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	
//	++vacuum.tick;
//	
//	if(vacuum.tick <= changeTick)
//	{
//		GPIO_SetBits(GPIOA,GPIO_Pin_0);
//	}
//	else if(vacuum.tick > changeTick && vacuum.tick <=max_tick)
//	{
//		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
//	}
//	else
//	{
//		vacuum.tick = 0 ;
//	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StartVacuum
*	功能说明: 开启吸尘器
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartVacuum(void)
{
	vacuum.tick = 0 ;
	vacuum.isRunning = true;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StopVacuum
*	功能说明: 关闭吸尘器
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopVacuum(void)
{
	vacuum.isRunning = false;
	vacuum.tick = 0 ;
	
	GPIO_ResetBits(GPIO_PORT_VACUUM,GPIO_PIN_VACUUM);
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitTimer1
*	功能说明: 初始化PWM，注意定时器是否需要重映射
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitTimer1(uint16_t arr,uint16_t psc)
{  
	
	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); /*Timer1重映射*/     	

	/*GPIO初始化*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/*定时器初始化*/
	TIM_TimeBaseStructure.TIM_Period 		= arr; 					
	TIM_TimeBaseStructure.TIM_Prescaler     = psc; 					
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;  	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
	
	/*
	PWM一共有两种模式，PWM1模式：CNT<CRRx为有效电平（不包括等于）。CNT>=CRRx为无效电平。PWM2模式相反。
	那么问题来了什么为有效电平？他又怎么确定？
	它是由TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;输出极性来定的。
	有效电平加输出极性就等于什么时候输出高电
	*/ 
	TIM_OCInitStructure.TIM_OCMode 		= TIM_OCMode_PWM1; 			
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;	

	/*TIM1，TIM8必须使用，否则在RTOS中无法使用，暂时不了解原因*/
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;	/* only for TIM1 and TIM8. */	
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;			/* only for TIM1 and TIM8. */		
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		/* only for TIM1 and TIM8. */
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;		/* only for TIM1 and TIM8. */
	
	/*CH1*/
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);	 
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	/*CH2*/
	TIM_OC2Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	/*CH3*/
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);	 
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	/*CH4*/
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);	 
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
  
	/*使能TIM1*/
	TIM_Cmd(TIM1, ENABLE); 
	
	/*TIM1，TIM8必须使用，其他定时器可使用或不使用*/
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	/*4个通道全部输出高电平*/
	TIM_SetCompare1(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare2(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare3(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare4(TIM1,CONSTANT_HIGH_PWM);
	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitTimer4
*	功能说明: 初始化PWM，注意定时器是否需要重映射
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitTimer4(uint16_t arr,uint16_t psc)
{  
	
	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); /*Timer1重映射*/     	

	/*GPIO初始化*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 |GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/*定时器初始化*/
	TIM_TimeBaseStructure.TIM_Period 		= arr; 					
	TIM_TimeBaseStructure.TIM_Prescaler     = psc; 					
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;  	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
	/*
	PWM一共有两种模式，PWM1模式：CNT<CRRx为有效电平（不包括等于）。CNT>=CRRx为无效电平。PWM2模式相反。
	那么问题来了什么为有效电平？他又怎么确定？
	它是由TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;输出极性来定的。
	有效电平加输出极性就等于什么时候输出高电
	*/ 
	TIM_OCInitStructure.TIM_OCMode 		= TIM_OCMode_PWM1; 			
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;	

	/*TIM1，TIM8必须使用，否则在RTOS中无法使用，暂时不了解原因*/
	#if 0
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;	/* only for TIM1 and TIM8. */	
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;			/* only for TIM1 and TIM8. */		
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		/* only for TIM1 and TIM8. */
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;		/* only for TIM1 and TIM8. */
	#endif
	
	/*CH1*/
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	/*CH2*/
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	/*CH3*/
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	/*CH4*/
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);	 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
	/*使能TIM*/
	TIM_Cmd(TIM4, ENABLE); 
	
	/*TIM1，TIM8必须使用，其他定时器可使用或不使用*/
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	
	/*4个通道全部输出高电平*/
	TIM_SetCompare1(TIM4,CONSTANT_HIGH_PWM);
	TIM_SetCompare2(TIM4,CONSTANT_HIGH_PWM);
	TIM_SetCompare3(TIM4,CONSTANT_HIGH_PWM);
	TIM_SetCompare4(TIM4,CONSTANT_HIGH_PWM);
	
}



