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
#if 0 /*没有红外协助的碰撞*/
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
	
#else /*有红外协助的碰撞*/
	
	Collision ret;
	
	if(GPIO_ReadInputDataBit(GPIO_PORT_LEFT,GPIO_PIN_LEFT) && GPIO_ReadInputDataBit(GPIO_PORT_RIGHT,GPIO_PIN_RIGHT))
	{
		ret = CollisionAll;
	}
	else if(GPIO_ReadInputDataBit(GPIO_PORT_LEFT,GPIO_PIN_LEFT) && bsp_AssistIsFaceObstacles())
	{
		ret = CollisionAll;
	}
	else if(GPIO_ReadInputDataBit(GPIO_PORT_RIGHT,GPIO_PIN_RIGHT) && bsp_AssistIsFaceObstacles())
	{
		ret = CollisionAll;
	}
	else if(GPIO_ReadInputDataBit(GPIO_PORT_LEFT,GPIO_PIN_LEFT))
	{
		ret = CollisionLeft;
	}
	else if(GPIO_ReadInputDataBit(GPIO_PORT_RIGHT,GPIO_PIN_RIGHT))
	{
		ret = CollisionRight;
	}
	else
	{
		ret = CollisionNone;
	}
	
	return ret;
	
#endif
}

/*
*********************************************************************************************************
*	函 数 名: bsp_CollisionDemo
*	功能说明: 碰撞测试
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_CollisionDemo(void)
{
	Collision ret = bsp_CollisionScan();
	
	switch(ret)
	{
		case CollisionLeft:
		{
			DEBUG("左边被碰撞了\r\n");
		}break;
		
		case CollisionRight:
		{
			DEBUG("右边被碰撞了\r\n");
		}break;
		
		case CollisionAll:
		{
			DEBUG("两边被碰撞了\r\n");
		}break;
		
		case CollisionNone:
		{
		
		}break;
	}
}



