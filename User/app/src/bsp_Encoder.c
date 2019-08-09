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

