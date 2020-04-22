#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1



typedef struct
{
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
	
	bool isNeedPlaySong;
	uint32_t isNeedPlaySongTick;
	uint32_t lastIsChargingTick;
	uint32_t lastIsTouchTick;
}Serach;


static Serach search;


/*
*********************************************************************************************************
*	函 数 名: bsp_StartSearchChargePile
*	功能说明: 开启寻找充电桩状态机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartSearchChargePile(void)
{
	bsp_IRD_StartWork();
	
	search.action = 0 ;
	search.delay = 0 ;
	search.isRunning = true;
	
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StopSearchChargePile
*	功能说明: 停止寻找充电桩状态机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopSearchChargePile(void)
{
	search.isRunning = false;
	search.action = 0 ;
	search.delay = 0 ;
	
	bsp_IRD_StopWork();
}	


/*
*********************************************************************************************************
*	函 数 名: bsp_SearchChargePile
*	功能说明: 寻找充电桩状态机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SearchChargePile(void)
{
	if(bsp_IsTouchChargePile())
	{
		/*播放开始充电*/
		if(search.isNeedPlaySong && (bsp_GetRunTime() - search.isNeedPlaySongTick >= 1000) ) /*这个时间判断避免了抖动播放开始充电*/
		{
			search.isNeedPlaySongTick = UINT32_T_MAX; /*给个最大时间刻度，下次自然不会满足*/
			bsp_SperkerPlay(Song22);
			search.isNeedPlaySong = false;
		}

		if(bsp_IsCharging())
		{
			bsp_SetLedState(AT_CHARGING);
			search.lastIsChargingTick = bsp_GetRunTime();
			DEBUG("is charging\r\n");
		}
		else
		{
			if(bsp_GetRunTime() - search.lastIsChargingTick >= 1000) /*这个时间判断避免了黄灯，绿灯闪烁*/
			{
				bsp_SetLedState(AT_CHARGE_DONE);
			}

			DEBUG("charge done\r\n");
		}
		
		search.lastIsTouchTick = bsp_GetRunTime();
	}
	else /*离桩状态需要立马更改，但是灯需要等会儿处理，不然会抖动*/
	{
		search.isNeedPlaySong = true;
		search.isNeedPlaySongTick = bsp_GetRunTime();
		
		if(bsp_GetRunTime() - search.lastIsTouchTick >= 500)
		{
			bsp_SetLedState(THREE_WHITE_ON);
			
			 search.lastIsTouchTick = UINT32_T_MAX;
		}
		
	}
}

	
/*
*********************************************************************************************************
*	函 数 名: bsp_IsTouchChargePile
*	功能说明: 获取充电反馈
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_IsTouchChargePile(void)
{
	if(GPIO_ReadInputDataBit(GPIO_PORT_CHARGE_TOUCH_PILE,GPIO_PIN_CHARGE_TOUCH_PILE))
	{
		return true ;
	}
	else
	{
		return false ;
	}
}


/*
*********************************************************************************************************
*	函 数 名: bsp_IsCharging
*	功能说明: 是否已经充满
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_IsCharging(void)
{
	if(GPIO_ReadInputDataBit(GPIO_PORT_CHARGE_IS_CHARGING,GPIO_PIN_CHARGE_IS_CHARGING)==0)
	{
		return true ;
	}
	else
	{
		return false ;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_IsChargeDone
*	功能说明: 是否已经充满
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_IsChargeDone(void)
{
	if(GPIO_ReadInputDataBit(GPIO_PORT_CHARGE_IS_DONE,GPIO_PIN_CHARGE_IS_DONE)==0)
	{
		return true ;
	}
	else
	{
		return false ;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitChargeIO
*	功能说明: 初始化充电相关的IO
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitChargeIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	
	/*上桩检测*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CHARGE_TOUCH_PILE;
	GPIO_Init(GPIO_PORT_CHARGE_TOUCH_PILE, &GPIO_InitStructure);
	
	/*正在充电中*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CHARGE_IS_CHARGING;
	GPIO_Init(GPIO_PORT_CHARGE_IS_CHARGING, &GPIO_InitStructure);
	
	/*充电完成*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CHARGE_IS_DONE;
	GPIO_Init(GPIO_PORT_CHARGE_IS_DONE, &GPIO_InitStructure);
	
}

