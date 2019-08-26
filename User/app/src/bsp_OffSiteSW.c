#include "bsp.h"

#define RCC_ALL_OFFSITE_SW 	(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE)

#define GPIO_PORT_OFFSITE_SW_L  GPIOC
#define GPIO_PIN_OFFSITE_SW_L   GPIO_Pin_9


#define GPIO_PORT_OFFSITE_SW_R  GPIOE
#define GPIO_PIN_OFFSITE_SW_R   GPIO_Pin_12


/*
*********************************************************************************************************
*	函 数 名: bsp_InitOffSiteSW
*	功能说明: 初始化离地开关使能引脚
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitOffSiteSW(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_OFFSITE_SW, ENABLE);

	/*
		初始状态先关闭
	*/
	bsp_SwOff(SW_5V_EN_CTRL);
	bsp_SwOff(SW_IR_POWER);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	/* 浮空输入 */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_OFFSITE_SW_L;
	GPIO_Init(GPIO_PORT_OFFSITE_SW_L, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	/* 浮空输入 */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_OFFSITE_SW_R;
	GPIO_Init(GPIO_PORT_OFFSITE_SW_R, &GPIO_InitStructure);

}


/*
*********************************************************************************************************
*	函 数 名: bsp_OffSiteGetState
*	功能说明: 返回当前离地开关状态
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
OffSiteState bsp_OffSiteGetState(void)
{
	OffSiteState ret ;
	
	if(!GPIO_ReadInputDataBit(GPIO_PORT_OFFSITE_SW_L,GPIO_PIN_OFFSITE_SW_L)&&
		!GPIO_ReadInputDataBit(GPIO_PORT_OFFSITE_SW_R,GPIO_PIN_OFFSITE_SW_R))
	{
		ret = OffSiteBoth;
	}
	else if(!GPIO_ReadInputDataBit(GPIO_PORT_OFFSITE_SW_L,GPIO_PIN_OFFSITE_SW_L))
	{
		ret = OffSiteLeft;
	}
	else if(!GPIO_ReadInputDataBit(GPIO_PORT_OFFSITE_SW_R,GPIO_PIN_OFFSITE_SW_R))
	{
		ret = OffSiteRight;
	}
	else
	{
		ret = OffSiteNone;
	}
	
	return ret;
}

