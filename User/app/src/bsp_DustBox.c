#include "bsp.h"

/* 防碰撞触动开关对应的RCC时钟 */
#define RCC_ALL_DUST_BOX 	(RCC_APB2Periph_GPIOG)

#define GPIO_PORT_DUST_BOX   GPIOG
#define GPIO_PIN_DUST_BOX	 GPIO_Pin_14





/*
*********************************************************************************************************
*	函 数 名: bsp_InitDustBox
*	功能说明: 初始化尘盒检测
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitDustBox(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_DUST_BOX, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; /* 上拉输入 */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_DUST_BOX;
	GPIO_Init(GPIO_PORT_DUST_BOX, &GPIO_InitStructure);
}


/*
*********************************************************************************************************
*	函 数 名: bsp_DustBoxGetState
*	功能说明: 返回尘盒状态
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
DustBoxState bsp_DustBoxGetState(void)
{
	DustBoxState ret ;
	
	if(GPIO_ReadInputDataBit(GPIO_PORT_DUST_BOX,GPIO_PIN_DUST_BOX) == 0)
	{
		ret = DustBoxInside;
	}
	else
	{
		ret = DustBoxOutside;
	}
	
	return ret ;
}


