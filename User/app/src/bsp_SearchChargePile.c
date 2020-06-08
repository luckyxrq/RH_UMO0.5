#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1

#define MAX_SEARCH_TICK         (1000*120)


/*前面的2个接收头*/
#define FRONT_RX_L      IR_CH2
#define FRONT_RX_R      IR_CH1

#define IR_RX_L      IR_CH3
#define IR_RX_R      IR_CH4

/*直走*/
#define CASE_RANDOM_0    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))
#define CASE_RANDOM_7    (!bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))
#define CASE_RANDOM_10    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))

/*向右划大弧线*/
#define CASE_RANDOM_11   ((bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_RIGHT)) && CASE_RANDOM_7)
/*向左划大弧线*/
#define CASE_RANDOM_12   ((bsp_IR_GetRev(IR_RX_L,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_RX_L,IR_TX_SITE_RIGHT)) && CASE_RANDOM_7)


#define CASE_RANDOM_3    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))
#define CASE_RANDOM_4    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))

#define CASE_RANDOM_5    (!bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT))
#define CASE_RANDOM_6    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))

#define CASE_RANDOM_8    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))
#define CASE_RANDOM_9    (!bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))

#define CASE_RANDOM_13   ((bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_RX_L,IR_TX_SITE_RIGHT)) && CASE_RANDOM_7)
				

#define CASE_STOP_RANDOM (CASE_RANDOM_11 || CASE_RANDOM_12 || CASE_RANDOM_0 || CASE_RANDOM_10 || CASE_RANDOM_3 || CASE_RANDOM_4 || CASE_RANDOM_5 || CASE_RANDOM_6 || CASE_RANDOM_8 || CASE_RANDOM_9)

typedef enum
{
	eNone = 0 ,          /*没有碰撞*/
	eHasSignalCollision, /*和充电桩碰上了后碰撞*/
	eNoSignalCollision   /*没有信号的碰撞，没有碰上充电桩，比如碰上了墙壁*/
}SearchChargePileCollision;

typedef struct
{
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
	
	uint32_t startTick;
	
	bool isNeedPlaySong;
	uint32_t isNeedPlaySongTick;
	uint32_t lastIsChargeDoneTick;
	uint32_t lastIsTouchTick;
	uint32_t lastIsNeedEdgeTick;
	
	bool isNeedFirstRunRandom;
	
	Collision collision;
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
	
	
	search.action = 0 ;
	search.delay = 0 ;
	search.startTick = xTaskGetTickCount();
	search.isNeedFirstRunRandom = true;
	search.isRunning = true;
	
	/*防止编译器警告*/

	bsp_IRD_StartWork();
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
	
	bsp_SetMotorSpeed(MotorLeft, 0);
	bsp_SetMotorSpeed(MotorRight,0);
	search.isRunning = false;
	search.isNeedFirstRunRandom = false;
	search.action = 0 ;
	search.delay = 0 ;
	
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
	
	bsp_IRD_StopWork();
}	



void bsp_DetectIsTouchChargePile(void)
{
	if(search.isRunning && bsp_IsTouchChargePile())
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		
		bsp_CloseAllStateRun();
		
		search.isRunning = false;
	}
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
	
	bsp_IsTouchChargePile();
	bsp_IsCharging();
	bsp_IsChargeDone();
	
	
	//DEBUG("%d %d  ---  %d %d\r\n",bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT),bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT),bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT),bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT));
	//DEBUG("%d %d  ---  %d %d\r\n",bsp_IR_GetRev(IR_RX_L,IR_TX_SITE_LEFT),bsp_IR_GetRev(IR_RX_L,IR_TX_SITE_RIGHT),bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_LEFT),bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_RIGHT));

	if(isCleanRunning())
		return;
	
	if(bsp_IsTouchChargePile())
	{
		/*播放开始充电*/
		if(search.isNeedPlaySong && (xTaskGetTickCount() >= search.isNeedPlaySongTick  && xTaskGetTickCount() - search.isNeedPlaySongTick >= 1000) ) /*这个时间判断避免了抖动播放开始充电*/
		{
			search.isNeedPlaySongTick = UINT32_T_MAX; /*给个最大时间刻度，下次自然不会满足*/
			bsp_SperkerPlay(Song22);
			search.isNeedPlaySong = false;
		}

		if(bsp_IsChargeDone())
		{
			
			if(xTaskGetTickCount() - search.lastIsChargeDoneTick >= 1000) /*这个时间判断避免了黄灯，绿灯闪烁*/
			{
				bsp_SetLedState(AT_CHARGE_DONE);
			}
		}
		else
		{
			bsp_SetLedState(AT_CHARGING);
			search.lastIsChargeDoneTick = xTaskGetTickCount();
		}
		
		search.lastIsTouchTick = xTaskGetTickCount();
	}
	else /*离桩状态需要立马更改，但是灯需要等会儿处理，不然会抖动*/
	{
		if((xTaskGetTickCount() >= search.lastIsTouchTick)  &&  (xTaskGetTickCount() - search.lastIsTouchTick >= 500))
		{
			 /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!治理需要根据扫地与否更改*/
			if( bsp_GetLedAppState() != AT_CLEAN && 
				bsp_GetLedAppState() != AT_SEARCH_CHARGE &&
				bsp_GetLedAppState() != THREE_WHITE_TOOGLE &&
			    bsp_GetLedAppState() != AT_LINK)
			{
				bsp_SetLedState(THREE_WHITE_ON);
			}
			
			search.isNeedPlaySongTick = xTaskGetTickCount();
			search.isNeedPlaySong = true;
			search.lastIsTouchTick = UINT32_T_MAX;
		}
		
	}
	
	
	if(!search.isRunning)
	{
		return;
	}
		

	/*计时，时间到了还没找到充电桩*/
	if(xTaskGetTickCount() - search.startTick >= MAX_SEARCH_TICK)
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_CloseAllStateRun();
		
		bsp_SetLedState(THREE_WHITE_ON); 
		bsp_SperkerPlay(Song24);
		
		return;
	}
	
	
	
	/*充电*/
	if(bsp_IsTouchChargePile() == true)
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_CloseAllStateRun();
		
		return ;
	}
	
	/*先随机清扫*/
	if(search.isNeedFirstRunRandom)
	{
		bsp_StartStrategyRandom();
		search.isNeedFirstRunRandom = false;
	}
	
	
	
	/*如果在执行随机清扫，则不执行后面的寻找充电桩*/
	if(bsp_IsStartStrategyRandom())
	{
		if(CASE_STOP_RANDOM) /*收到了信号就执行后面的寻找充电桩*/
		{
			DEBUG("退出随机\r\n");
			
			bsp_SetMotorSpeed(MotorLeft,0);
			bsp_SetMotorSpeed(MotorRight,0);
			bsp_StopStrategyRandom();
		}
		return;
	}
	
	
	switch(search.action)
	{
		case 0:
		{
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,6);
			
			search.action++;
		}break;
		
		case 1:
		{
			/*首先判断碰撞*/
			search.collision = bsp_CollisionScan();
			
			//if(ret != CollisionNone || bsp_CliffIsDangerous(CliffLeft) || bsp_CliffIsDangerous(CliffMiddle) || bsp_CliffIsDangerous(CliffRight))
			if(search.collision != CollisionNone )
			{
				/*不管如何碰到了就后退，在后退的过程中再来调节轮子*/
				bsp_SetMotorSpeed(MotorLeft, -3);
				bsp_SetMotorSpeed(MotorRight,-3);
				
				search.delay = xTaskGetTickCount();
				search.action++;
			}
			else
			{
				#define CASE_RANDOM_15   (bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_CENTER) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_CENTER) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_CENTER))
				
				if(CASE_RANDOM_11) /*大弧线 侧面收到充电桩的2个窄角中的任何一个 ， 并且前面没有收到信号*/
				{
					bsp_SetMotorSpeed(MotorLeft, 8);
					bsp_SetMotorSpeed(MotorRight,0);
				}
				else if(CASE_RANDOM_12) /*大弧线 侧面收到充电桩的2个窄角中的任何一个 ， 并且前面没有收到信号*/
				{
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,8);
				}
				else if(CASE_RANDOM_0 ||  CASE_RANDOM_10) /*（左能同时左右 并且 右能同时左右） 或者  （左收左左不能右  右收右右不能收左）*/
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,3);
				}
				else if(CASE_RANDOM_3 || CASE_RANDOM_6 || CASE_RANDOM_8)
				{
					/*左能收左右，右能收右不能收左*/
					/*左能收右，并且右不能收左也不能收右*/
					/*左能同时左右 右既不能左也不能右*/
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,4);
				}
				else if(CASE_RANDOM_4 || CASE_RANDOM_5 || CASE_RANDOM_9)
				{
					/*左能左  左不能右 并且 右能左*/
					/*左既不能左也不能右  右能左*/
					/*左既不能左也不能右，右能同时左右*/
					bsp_SetMotorSpeed(MotorLeft, 5);
					bsp_SetMotorSpeed(MotorRight,3);
				}
				
				else if(CASE_RANDOM_15)
				{
					/*右边接收能收到广角  前面两个收不到广角*/
					bsp_SetMotorSpeed(MotorLeft, 5);
					bsp_SetMotorSpeed(MotorRight,3);
				}
				
				else if(CASE_RANDOM_7) /*左收不到左右任何一个 并且 右收不到左右任何一个*/
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,3);
				}
			}
		}break;
		

		case 2:
		{
			if(xTaskGetTickCount() - search.delay >= 2000)
			{
				search.action = 0 ;
			}
			else
			{
				if(CASE_RANDOM_3 || CASE_RANDOM_6 || CASE_RANDOM_8)
				{
					bsp_SetMotorSpeed(MotorLeft, -5);
					bsp_SetMotorSpeed(MotorRight,-2);
				}
				else if(CASE_RANDOM_4 || CASE_RANDOM_5 || CASE_RANDOM_9)
				{
					bsp_SetMotorSpeed(MotorLeft, -2);
					bsp_SetMotorSpeed(MotorRight,-5);
				}
			}
		}break;

	}
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitIO
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

