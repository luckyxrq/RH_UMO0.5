#include "bsp.h"

/* 编码器口对应的RCC时钟 */
#define RCC_ALL_ENCODER 	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE)

#define GPIO_PORT_LEFT   GPIOA
#define GPIO_PIN_LEFT	 GPIO_Pin_15

#define GPIO_PORT_RIGHT  GPIOE
#define GPIO_PIN_RIGHT	 GPIO_Pin_3



static Encoder encoder;
static uint32_t interruptCount = 0 ;

static void bsp_InitEncoderIO(void);
static void bsp_InitEncoderTick(void);
static void bsp_EncoderCalcSpeed(EncoderSN sn);

/*
*********************************************************************************************************
*	函 数 名: bsp_InitEncoder
*	功能说明:初始化编码器需要的定时器和IO口。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitEncoder(void)
{
	bsp_InitEncoderIO();
	bsp_InitEncoderTick();
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitEncoderIO
*	功能说明:初始化编码器IO口。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitEncoderIO(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_ALL_ENCODER, ENABLE);
				  
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_PIN_LEFT;
	GPIO_Init(GPIO_PORT_LEFT, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_PIN_RIGHT;
	GPIO_Init(GPIO_PORT_RIGHT, &GPIO_InitStructure); 
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitEncoderTick
*	功能说明:初始化编码器定时器。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitEncoderTick(void)
{
	bsp_SetTIMforInt(TIM7, ENCODER_INTERRUPT_FREQUENCY, 1, 0);
}


/*
*********************************************************************************************************
*	函 数 名: bsp_EncoderCalcSpeed
*	功能说明: 通过编码器计算速度。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_EncoderCalcSpeed(EncoderSN sn)
{
	encoder.speed[sn] = (float)encoder.risingCount[sn] / (float)Ratio *  (float)PERIMETER / 0.3F;
	encoder.risingCount[sn] = 0 ;
	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_EncoderGetSpeed
*	功能说明: 返回编码器反馈的电机运行的速度
*	形    参: 编码器编号
*	返 回 值: 无
*********************************************************************************************************
*/
float bsp_EncoderGetSpeed(EncoderSN sn)
{
	return encoder.speed[sn];
}


/*
*********************************************************************************************************
*	函 数 名: TIM7_IRQHandler
*	功能说明:每100us进入一次中断，如果捕获到一个上升沿，则脉冲个数加1
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void TIM7_IRQHandler(void)
{
	if(RESET != TIM_GetITStatus(TIM7, TIM_IT_Update)) 
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update); 
		++interruptCount;
		
		
		/*左脉冲计数*/
		if(GPIO_ReadInputDataBit(GPIO_PORT_LEFT,  GPIO_PIN_LEFT) == 0)
		{
			encoder.isReadyRising[EncoderLeft] = true;
		}
		else
		{
			if(encoder.isReadyRising[EncoderLeft])
			{
				encoder.isReadyRising[EncoderLeft] = false;
				++encoder.risingCount[EncoderLeft];
				encoder.odometer[0] += (bsp_MotorGetDir(MotorLeft)==Forward ? 1 : -1);/*里程计*/
			}
		}
		
		/*右脉冲计数*/
		if(GPIO_ReadInputDataBit(GPIO_PORT_RIGHT,  GPIO_PIN_RIGHT) == 0)
		{
			encoder.isReadyRising[EncoderRight] = true;
		}
		else
		{
			if(encoder.isReadyRising[EncoderRight])
			{
				encoder.isReadyRising[EncoderRight] = false;
				++encoder.risingCount[EncoderRight];
				encoder.odometer[1] += (bsp_MotorGetDir(MotorRight)==Forward ? 1 : -1);/*里程计*/
			}
		}

		/*计算速度*/
		if(interruptCount >= CALC_T)
		{
			interruptCount = 0 ;
			
			bsp_EncoderCalcSpeed(EncoderLeft);
			bsp_EncoderCalcSpeed(EncoderRight);
		}

	}
}



int32_t bsp_encoderGetOdometer(MotorSN sn)
{
	int32_t odometer = 0 ;
	switch(sn)
	{
		case MotorLeft:
		{
			odometer = encoder.odometer[0];
		}break;
		
		case MotorRight:
		{
			odometer = encoder.odometer[1];
		}break;
	}
	
	return  odometer;
	
}







/////////////////////////////////////////////////////////////////


/* 定义例程名和例程发布日期 */
#define EXAMPLE_NAME	"V4-004_EXIT外部中断例程"
#define EXAMPLE_DATE	"2015-08-30"
#define DEMO_VER		"1.0"





/*
*********************************************************************************************************
*	函 数 名: EXTI0_Config
*	功能说明: 配置外部中断。K1、K2、K3 中断.
*				K1键 PC13 ：下降沿触发
*				K2键 PA0  : 上升沿触发
*				K3键 PG8  : 下降沿触发
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void EXTI_Config(void)
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
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
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
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
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

		DEBUG("EXTI_Line15");
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
		
		DEBUG("EXTI_Line3");
	}
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/



