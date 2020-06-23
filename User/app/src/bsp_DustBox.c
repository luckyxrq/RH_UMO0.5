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
	
	if(GPIO_ReadInputDataBit(GPIO_PORT_DUST_BOX,GPIO_PIN_DUST_BOX))
	{
		ret = DustBoxOutside;
	}
	else
	{
		ret = DustBoxInside ;
	}
	
	return ret ;
}


typedef struct
{
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
	volatile uint16_t cnt;
}DustBoxProc;

static DustBoxProc dustBoxProc;


void bsp_StartDustBoxProc(void)
{
	dustBoxProc.action = 0 ;
	dustBoxProc.delay = 0 ;
	dustBoxProc.isRunning = true;
	dustBoxProc.cnt = 0;
}

void bsp_StopDustBoxProc(void)
{
	dustBoxProc.isRunning = false;
	dustBoxProc.action = 0 ;
	dustBoxProc.delay = 0 ;
}



/*
*********************************************************************************************************
*	函 数 名: bsp_OffSiteProc
*	功能说明: 离地开关处理函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_DustBoxProc(void)
{
	DustBoxState state;
	
	if(!dustBoxProc.isRunning)
		return;
	
	switch(dustBoxProc.action)
	{
		case 0:
		{
			state = bsp_DustBoxGetState();
			if(state == DustBoxOutside)
			{
				dustBoxProc.cnt++;
				if(dustBoxProc.cnt > 300)
				{
					bsp_OffsiteSuspend();
					/*尘盒取出*/
					bsp_SperkerPlay(Song9);
					dustBoxProc.action++;
				} 
				else
				{
					dustBoxProc.cnt = 0 ;
				}
			}
		}break;
		
		case 1: /*等待成盒装回*/
		{
			state = bsp_DustBoxGetState();
			if(state == DustBoxInside)
			{
				if(dustBoxProc.cnt > 0) dustBoxProc.cnt--;
				if(dustBoxProc.cnt == 0)
				{					
					bsp_SperkerPlay(Song10);
					dustBoxProc.action = 0 ;
				}
				
			}
		}break;
	}
}

