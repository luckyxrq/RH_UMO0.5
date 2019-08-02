#include "bsp.h"

/* 口对应的RCC时钟 */
#define RCC_ALL_PULSE 	(RCC_APB2Periph_GPIOB)

#define GPIO_PORT_PULSE  GPIOB
#define GPIO_PIN_PULSE	 GPIO_Pin_0


/*
*********************************************************************************************************
*	函 数 名: bsp_InitPinPulse
*	功能说明: 初始化脉冲引脚，用于示波器协助检测
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitPinPulse(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_ALL_PULSE, ENABLE );	 
                      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_PULSE;
	GPIO_Init(GPIO_PORT_PULSE, &GPIO_InitStructure);	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_SendPulse
*	功能说明: 发送一个10US的高电平脉冲
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SendPulse(void)
{
	GPIO_SetBits(GPIO_PORT_PULSE,GPIO_PIN_PULSE);
	bsp_DelayUS(10);
	GPIO_ResetBits(GPIO_PORT_PULSE,GPIO_PIN_PULSE);
}

