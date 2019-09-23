#include "bsp.h"

#define RCC_ALL_RCC 	(RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO)

/*用于唤醒离开stop模式的按键*/
#define GPIO_PORT_K    GPIOE
#define GPIO_PIN_K     GPIO_Pin_7

/*
*********************************************************************************************************
*	函 数 名: bsp_InitKeyStopMODE
*	功能说明: 离开stop模式的按键
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitKeyStopMODE(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_ALL_RCC, ENABLE);

	/* 配置引脚 */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIO_PORT_K, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource7);

    /* 配置外部中断事件 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	/* 16个抢占式优先级，0个响应式优先级 */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_EnterStopMODE
*	功能说明: 进入stop模式
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_EnterStopMODE(void)
{
	/*
		1. 停止模式是在Cortex-M3的深睡眠模式基础上结合了外设的时钟控制机制，在停止模式下电压
		调节器可运行在正常或低功耗模式。此时在1.8V供电区域的的所有时钟都被停止， PLL、 HSI和
		HSE的RC振荡器的功能被禁止， SRAM和寄存器内容被保留下来。
		2. 在停止模式下，所有的I/O引脚都保持它们在运行模式时的状态。
		3. 一定要关闭滴答定时器，实际测试发现滴答定时器中断也能唤醒停机模式。
	*/
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  /* 关闭滴答定时器 */  
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFE);				
	portENTER_CRITICAL();

	/* 
		1、当一个中断或唤醒事件导致退出停止模式时， HSI RC振荡器被选为系统时钟。
		2、退出低功耗的停机模式后，需要重新配置使用HSE。		
	*/
	RCC_HSEConfig(RCC_HSE_ON);
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET){}

	RCC_PLLCmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	while (RCC_GetSYSCLKSource() != 0x08){}
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; /* 使能滴答定时器 */  
	portEXIT_CRITICAL();
}


void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line7) == SET)
	{	
		bsp_InitEncoder();
		bsp_SwOn(SW_IR_POWER);
		bsp_SwOn(SW_MOTOR_POWER);
		
		EXTI_ClearITPendingBit(EXTI_Line7); /* 清除中断标志位 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_DISABLE_ALL_EXIT
*	功能说明: 关闭所有的外部中断
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_DISABLE_ALL_EXIT(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

