#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1


#define CH1_HAVE_ANG_SIGNAL   (bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT)||bsp_IR_GetRev(IR_CH1,IR_TX_SITE_CENTER)||bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
#define CH2_HAVE_ANG_SIGNAL   (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT)||bsp_IR_GetRev(IR_CH2,IR_TX_SITE_CENTER)||bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
#define CH3_HAVE_ANG_SIGNAL   (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT)||bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER)||bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT))
#define CH4_HAVE_ANG_SIGNAL   (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT)||bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER)||bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)) 
		     
/*寻找冲端庄之前先旋转，这里是旋转停止的原因*/
#define CCW_STOP_BY_CH1_2      1
#define CCW_STOP_BY_CH3        2
#define CCW_STOP_BY_CH4        3
#define CCW_STOP_BY_OUTTIME    4

typedef struct
{
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
	
	float angle;
	
	bool isNeedPlaySong;
	uint32_t isNeedPlaySongTick;
	uint32_t lastIsChargingTick;
	uint32_t lastIsTouchTick;
	uint32_t lastIsNeedEdgeTick;
	
	
	uint8_t ccwSearchStopBy;
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
	
	//bsp_IRD_StopWork();
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
		bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
		bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));

		bsp_SperkerPlay(Song22);
		while(1);
	}
	
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
			bsp_SetLedState(THREE_WHITE_ON);   /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!治理需要根据扫地与否更改*/
			
			 search.lastIsTouchTick = UINT32_T_MAX;
		}
		
	}
	
	
	if(!search.isRunning)
		return;
	
	/*检测附近没有充电桩信号*/
	if(!(CH1_HAVE_ANG_SIGNAL || CH2_HAVE_ANG_SIGNAL || CH3_HAVE_ANG_SIGNAL || CH4_HAVE_ANG_SIGNAL))
	{
		search.lastIsNeedEdgeTick = bsp_GetRunTime(); 
	}
	
	if(bsp_GetRunTime() - search.lastIsNeedEdgeTick >= 10*1000)
	{
		bsp_StopSearchChargePile();
		bsp_StartEdgewiseRun();
		return ;
	}
	
	switch(search.action)
	{
		case 0:
		{
			/*直接旋转*/
			bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+100));
			search.angle = bsp_AngleRead();
			search.delay = bsp_GetRunTime();
			++search.action;
		}break;
		
		case 1:
		{
			if(CH1_HAVE_ANG_SIGNAL || CH2_HAVE_ANG_SIGNAL)
			{
				search.ccwSearchStopBy = CCW_STOP_BY_CH1_2;
				
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				++search.action;
			}
			else if(CH3_HAVE_ANG_SIGNAL)
			{
				search.ccwSearchStopBy = CCW_STOP_BY_CH3;
				
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				++search.action;
			}
			else if(CH4_HAVE_ANG_SIGNAL)
			{
				search.ccwSearchStopBy = CCW_STOP_BY_CH4;
				
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				++search.action;
			}
			else if(((bsp_GetRunTime()-search.delay)>=3000 && abs(bsp_AngleRead()-search.angle)<=10.0F))
			{
				search.ccwSearchStopBy = CCW_STOP_BY_OUTTIME;
				
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				++search.action;
			}
			
			
		}break;
		
		case 2:
		{
			if(search.ccwSearchStopBy == CCW_STOP_BY_CH3) /*此时应该转90度再去寻找*/
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+100));
				
				search.angle =  bsp_AngleAdd(bsp_AngleRead(),90);
				search.delay = bsp_GetRunTime();
				
				++search.action;
			}
			else
			{
				search.action = 4 ;
			}
		}break;
		
		case 3:
		{
			if(((bsp_GetRunTime()-search.delay)>=1000 && abs(bsp_AngleRead()-search.angle)<=10.0F))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				++search.action;
			}
		}break;
		
		case 4:
		{
			if(bsp_CollisionScan() != CollisionNone)
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				search.delay = bsp_GetRunTime();
				++search.action;
			}
			/*前面2个，都能收到左右发射*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)
				&& bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			/*前面2个，各收各*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			
			
			/*1号能收到2个 ,2号能收到左边*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			
			/*2号能收到2个 ,1号能收到右边*/
			else if(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			
			
			/*1，2号都能收到左边，都不能收到右边*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && 
				!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(220));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			/*1，2号都能收到右边，都不能收到左边*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && 
				!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(220));
			}

			
			/*1号不能同时收到左右发射，2能同时收到左右发射*/
			else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(220));
			}
			/*1号能同时收到左右发射，2不能同时收到左右发射*/
			else if((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(220));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			/*侧面4号能收到广角和左或右，正面1,2哈不能收到任何,原地旋转*/
			else if(bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			}
			/*侧面3号能收到广角和左或右，正面1,2哈不能收到任何,原地旋转*/
			else if(bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			/*1，2都不能同时收到左右发射*/
			else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
		}break;
		
		case 5:
		{
			if(bsp_GetRunTime() - search.delay <= 2000)
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-200));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-200));
			}
			else
			{
				/*前面2个，都能收到左右发射*/
				if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)
					&& bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
				}
				/*前面2个，各收各*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
				}
				
				
				/*1号能收到2个 ,2号能收到左边*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
				}
				
				/*2号能收到2个 ,1号能收到右边*/
				else if(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
				}
				
				
				/*1，2号都能收到左边，都不能收到右边*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && 
					!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-220));
				}
				/*1，2号都能收到右边，都不能收到左边*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && 
					!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-220));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				}

				
				/*1号不能同时收到左右发射，2能同时收到左右发射*/
				else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-220));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				}
				/*1号能同时收到左右发射，2不能同时收到左右发射*/
				else if((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-220));
				}
				/*侧面4号能收到广角和左或右，正面1,2哈不能收到任何,原地旋转*/
				else if(bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				}
				/*侧面3号能收到广角和左或右，正面1,2哈不能收到任何,原地旋转*/
				else if(bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT)))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				}
				/*1，2都不能同时收到左右发射*/
				else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
				}
				
				if(bsp_GetRunTime() - search.delay >= 3000)
				{
					--search.action;
				}
			}
		}break;
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

