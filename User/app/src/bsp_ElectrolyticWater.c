#include "bsp.h"

#define RCC_ALL_ELECTROLYTIC_WATER 	(RCC_APB2Periph_GPIOB)

#define GPIO_PORT_ELECTROLYTIC_WATER  GPIOB
#define GPIO_PIN_ELECTROLYTIC_WATER	  GPIO_Pin_3


#define ELECTROLYTIC_WATER_ON()      GPIO_SetBits(GPIO_PORT_ELECTROLYTIC_WATER,GPIO_PIN_ELECTROLYTIC_WATER)
#define ELECTROLYTIC_WATER_OFF()     GPIO_ResetBits(GPIO_PORT_ELECTROLYTIC_WATER,GPIO_PIN_ELECTROLYTIC_WATER)

/*
*********************************************************************************************************
*	函 数 名: bsp_InitLed
*	功能说明: 配置LED指示灯相关的GPIO,  该函数被 bsp_Init() 调用。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitElectrolyticWater(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_ELECTROLYTIC_WATER, ENABLE);

	ELECTROLYTIC_WATER_OFF();
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* 推挽输出模式 */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_ELECTROLYTIC_WATER;
	GPIO_Init(GPIO_PORT_ELECTROLYTIC_WATER, &GPIO_InitStructure);
	
	ELECTROLYTIC_WATER_OFF();
}

