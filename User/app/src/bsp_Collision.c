#include "bsp.h"


/* 防碰撞触动开关对应的RCC时钟 */
#define RCC_ALL_ANTICOLLISION 	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_LEFT   GPIOA
#define GPIO_PIN_LEFT	 GPIO_Pin_5

#define GPIO_PORT_RIGHT  GPIOF
#define GPIO_PIN_RIGHT	 GPIO_Pin_8




/*
*********************************************************************************************************
*	函 数 名: bsp_InitCollision
*	功能说明: 初始化碰撞检测引脚
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitCollision(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_ANTICOLLISION, ENABLE);


	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; /* 上拉输入，但是闲时被外部拉低，碰撞时恢复高电平 */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LEFT;
	GPIO_Init(GPIO_PORT_LEFT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RIGHT;
	GPIO_Init(GPIO_PORT_RIGHT, &GPIO_InitStructure);
}


/*
*********************************************************************************************************
*	函 数 名: bsp_CollisionScan
*	功能说明: 返回碰撞结果
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
Collision bsp_CollisionScan(void)
{
	uint8_t ret = 0 ;
	
	/* 左边撞上了 */
	if(GPIO_ReadInputDataBit(GPIO_PORT_LEFT,GPIO_PIN_LEFT))
	{
		ret |= 1<<0;
	}
	
	
	/* 右边撞上了 */
	if(GPIO_ReadInputDataBit(GPIO_PORT_RIGHT,GPIO_PIN_RIGHT))
	{
		ret |= 1<<1;
	}
	
	if(ret == 0x00)
	{
		return CollisionNone;
	}
	else if(ret == 0x01)
	{
		return CollisionLeft;
	}
	else if(ret == 0x02)
	{
		return CollisionRight;
	}
	else if(ret == 0x03)
	{
		return CollisionAll;
	}
	else
	{
		WARNING("不合理的碰撞结果\r\n");
		return CollisionNone;
	}
	
}



/*
*********************************************************************************************************
*	函 数 名: bsp_PrintCollision
*	功能说明: 打印碰撞信息
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_PrintCollision(void)
{
	if(bsp_CollisionScan() == CollisionLeft)
	{
		DEBUG("左边\r\n");
	}
	else if(bsp_CollisionScan() == CollisionRight)
	{
		DEBUG("右边\r\n");
	}
	else if(bsp_CollisionScan() == CollisionAll)
	{
		DEBUG("两边\r\n");
	}
}


