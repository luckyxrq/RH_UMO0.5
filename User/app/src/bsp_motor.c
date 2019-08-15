#include "bsp.h"



static void bsp_InitTimer1(uint16_t arr,uint16_t psc);

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
	/*初始化PWM 10KHZ，用于驱动电机*/
	bsp_InitTimer1(7199,0); 
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
	PWM一共有两种模式，PWM1模式：CNT<CRRx为有效电平。CNT>CRRx为无效电平。PWM2模式相反。
	那么问题来了什么为有效电平？他又怎么确定？
	它是由TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;输出极性来定的。
	有效电平加输出极性就等于什么时候输出高电
	*/ 
	TIM_OCInitStructure.TIM_OCMode 		= TIM_OCMode_PWM2; 			
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;	

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
	
	TIM_SetCompare1(TIM1,1000);
	TIM_SetCompare2(TIM1,5000);

}

