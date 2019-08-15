#include "bsp.h"

/*
**********************************************************************************************************
											变量声明
**********************************************************************************************************
*/
static Encoder encoder[ENCODER_NUM];

/*
**********************************************************************************************************
											函数声明
**********************************************************************************************************
*/
static void bsp_InitGPIO(void);
static void bsp_EXTI_Config(void);

/*
*********************************************************************************************************
*	函 数 名: bsp_InitEncoder
*	功能说明: 初始化编码器
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitEncoder(void)
{
	UNUSED(encoder);
	
	bsp_InitGPIO();
}


/*
*********************************************************************************************************
*	函 数 名: bsp_EncoderGetTotalMileage
*	功能说明: 获取总里程脉冲数，无正负
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
uint32_t bsp_EncoderGetTotalMileage(EncoderSN sn)
{
	return encoder[sn].totalMileage;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_EncoderGetPulseT
*	功能说明: 编码器M测速法，周期性的调用，5MS，10MS调用一次均可，理论此值有正负，但是只有一个霍尔，硬件层面
*             无法判断正负 
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
int32_t bsp_EncoderGetPulseT(EncoderSN sn)
{
	int32_t pulseCnt = 0 ;
	
	pulseCnt = encoder[sn].pulseT;
	encoder[sn].pulseT = 0 ;
	
	return pulseCnt;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitGPIO
*	功能说明: 初始化编码器IO扣，扫地机每个轮子编码器只有一个活儿传感器，没有正负
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitGPIO(void)
{
	bsp_EXTI_Config();
}


/*
*********************************************************************************************************
*	函 数 名: bsp_EXTI_Config
*	功能说明: 配置外部中断。编码器已经接了上拉电阻，单片机配置为浮空输入，2倍频捕获脉冲，上下江沿触发
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_EXTI_Config(void)
{
	/* 配置PA15 */
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef   EXTI_InitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;		
		
		/* 使能 GPIO 时钟 */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

		/* 配置 PA0 为输入浮空模式 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* 使能 AFIO 时钟 */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

		/* Connect EXTI0 Line to PA.00 pin */
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);

		/* Configure EXTI0 line */
		EXTI_InitStructure.EXTI_Line = EXTI_Line15;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  	
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Enable and set EXTI0 Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	
	/* 配置 PE3 外部中断 */
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef   EXTI_InitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;			
		
		/* 使能 GPIO 时钟 */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

		/* 配置 PC13 为输入浮空模式 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOE, &GPIO_InitStructure);

		/* 使能 AFIO 时钟 */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

		/* Connect EXTI13 Line to PA.00 pin */
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource3);

		/* Configure EXTI13 line */
		EXTI_InitStructure.EXTI_Line = EXTI_Line3;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  	/* 上升沿触发 */
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Enable and set EXTI13 Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}			
}


/*
*********************************************************************************************************
*	函 数 名: EXTI9_5_IRQHandler
*	功能说明: 外部中断服务程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void EXTI15_10_IRQHandler(void)
{

	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line15);
		
		++encoder[EncoderLeft].totalMileage; /*总里程脉冲数加1，无正负*/
	}
}


/*
*********************************************************************************************************
*	函 数 名: EXTI9_5_IRQHandler
*	功能说明: 外部中断服务程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void EXTI3_IRQHandler(void)
{

	if (EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);
		
		++encoder[EncoderRight].totalMileage; /*总里程脉冲数加1，无正负*/
	}
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/



