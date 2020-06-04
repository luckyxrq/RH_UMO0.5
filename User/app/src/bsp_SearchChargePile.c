#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1


#define STRAIGHT_SPEED_FAST      12
#define STRAIGHT_SPEED_SLOW      3

#define TURN_RIGHT_SPEED_FAST_L  5
#define TURN_RIGHT_SPEED_FAST_R  4

//
#define TURN_RIGHT_SPEED_SLOW_L  6
#define TURN_RIGHT_SPEED_SLOW_R  1


#define TURN_LEFT_SPEED_FAST_L   3
#define TURN_LEFT_SPEED_FAST_R   5
       
//	   
#define TURN_LEFT_SPEED_SLOW_L   1
#define TURN_LEFT_SPEED_SLOW_R   6

#define PIROUETTE_SPEED          4

#define BACKWARD_SPEED           -4

#define ROTATE_CCW_SPEED_L             -5
#define ROTATE_CCW_SPEED_R             5

#define MAX_SEARCH_TICK         (1000*120)

#define CORRECTION_ANGLE_TIME    200

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
static void bsp_SearchRunStraightFast(void);
static void bsp_SearchRunStraightSlow(void);
static void bsp_SearchTurnRightFast(void)  ;
static void bsp_SearchTurnRightSlow(void)  ;
static void bsp_SearchTurnLeftFast(void)   ;
static void bsp_SearchTurnLeftSlow(void)   ;
static void bsp_PirouetteCW(void)          ;
static void bsp_PirouetteCCW(void)         ;
static void bsp_GoBackward(void)           ;

static void bsp_SearchRunStraightFastBack(void);
static void bsp_SearchRunStraightSlowBack(void);
static void bsp_SearchTurnRightFastBack(void);
static void bsp_SearchTurnRightSlowBack(void);
static void bsp_SearchTurnLeftFastBack(void);
static void bsp_SearchTurnLeftSlowBack(void);


static void bsp_RotateCCW(void);
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
	UNUSED(bsp_SearchRunStraightFast);
	UNUSED(bsp_SearchRunStraightSlow);
	UNUSED(bsp_SearchTurnRightFast)  ;
	UNUSED(bsp_SearchTurnRightSlow)  ;
	UNUSED(bsp_SearchTurnLeftFast)   ;
	UNUSED(bsp_SearchTurnLeftSlow)   ;
	UNUSED(bsp_IsCharging);
	UNUSED(bsp_IsChargeDone);
	
	UNUSED(bsp_SearchRunStraightFastBack);
	UNUSED(bsp_SearchRunStraightSlowBack);
	UNUSED(bsp_SearchTurnRightFastBack);
	UNUSED(bsp_SearchTurnRightSlowBack);
	UNUSED(bsp_SearchTurnLeftFastBack);
	UNUSED(bsp_SearchTurnLeftSlowBack);
	
	UNUSED(bsp_RotateCCW);
	
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
		//DEBUG("接触桩:%s 充电中:%s 充满:%s\r\n",bsp_IsTouchChargePile()?"true":"false",bsp_IsCharging()?"true":"false",bsp_IsChargeDone()?"true":"false");
		
		
		
		
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
			bsp_SetMotorSpeed(MotorLeft, 8);
			bsp_SetMotorSpeed(MotorRight,8);
			
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
				bsp_GoBackward();
				
				search.delay = xTaskGetTickCount();
				search.action++;
			}
			else
			{
				
				
				
				if(CASE_RANDOM_11)
				{
					bsp_SetMotorSpeed(MotorLeft, 7);
					bsp_SetMotorSpeed(MotorRight,0);
				}
				else if(CASE_RANDOM_12)
				{
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,7);
				}
				else if(CASE_RANDOM_0 || CASE_RANDOM_7 || CASE_RANDOM_10)
				{
					bsp_SetMotorSpeed(MotorLeft, 6);
					bsp_SetMotorSpeed(MotorRight,6);
				}
				else if(CASE_RANDOM_3 || CASE_RANDOM_6 || CASE_RANDOM_8)
				{
					bsp_SetMotorSpeed(MotorLeft, 5);
					bsp_SetMotorSpeed(MotorRight,7);
				}
				else if(CASE_RANDOM_4 || CASE_RANDOM_5 || CASE_RANDOM_9)
				{
					bsp_SetMotorSpeed(MotorLeft, 7);
					bsp_SetMotorSpeed(MotorRight,5);
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
					bsp_SetMotorSpeed(MotorLeft, -7);
					bsp_SetMotorSpeed(MotorRight,-5);
				}
				else if(CASE_RANDOM_4 || CASE_RANDOM_5 || CASE_RANDOM_9)
				{
					bsp_SetMotorSpeed(MotorLeft, -5);
					bsp_SetMotorSpeed(MotorRight,-7);
				}
			}
		}break;

	}
}


/*
*********************************************************************************************************
*	函 数 名: bsp_SearchRunStraightFast
*	功能说明: 快速直行
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchRunStraightFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, STRAIGHT_SPEED_FAST);
	bsp_SetMotorSpeed(MotorRight,STRAIGHT_SPEED_FAST);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SearchRunStraightSlow
*	功能说明: 慢速直行
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchRunStraightSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, STRAIGHT_SPEED_SLOW);
	bsp_SetMotorSpeed(MotorRight,STRAIGHT_SPEED_SLOW);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SearchTurnRightFast
*	功能说明: 快速右转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchTurnRightFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_RIGHT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,TURN_RIGHT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SearchTurnRightSlow
*	功能说明: 慢速右转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchTurnRightSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_RIGHT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,TURN_RIGHT_SPEED_SLOW_R);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SearchTurnLeftFast
*	功能说明: 快速左转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchTurnLeftFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_LEFT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,TURN_LEFT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SearchTurnLeftSlow
*	功能说明: 慢速左转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchTurnLeftSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_LEFT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,TURN_LEFT_SPEED_SLOW_R);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_PirouetteCW
*	功能说明: 原地顺时针旋转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_PirouetteCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, PIROUETTE_SPEED);
	bsp_SetMotorSpeed(MotorRight,0);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_PirouetteCCW
*	功能说明: 原地逆时针旋转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_PirouetteCCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, 0);
	bsp_SetMotorSpeed(MotorRight,PIROUETTE_SPEED);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GoBackward
*	功能说明: 碰到障碍物后，快速倒退
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_GoBackward(void)
{
	bsp_SetMotorSpeed(MotorLeft, BACKWARD_SPEED);
	bsp_SetMotorSpeed(MotorRight,BACKWARD_SPEED);
}



/*
*********************************************************************************************************
*	函 数 名: bsp_SearchRunStraightFastBack
*	功能说明: 快速直行
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchRunStraightFastBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -STRAIGHT_SPEED_FAST);
	bsp_SetMotorSpeed(MotorRight,-STRAIGHT_SPEED_FAST);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SearchRunStraightSlowBack
*	功能说明: 慢速直行
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchRunStraightSlowBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -STRAIGHT_SPEED_SLOW);
	bsp_SetMotorSpeed(MotorRight,-STRAIGHT_SPEED_SLOW);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SearchTurnRightFastBack
*	功能说明: 快速右转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchTurnRightFastBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -TURN_RIGHT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,-TURN_RIGHT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SearchTurnRightSlowBack
*	功能说明: 慢速右转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchTurnRightSlowBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -TURN_RIGHT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,-TURN_RIGHT_SPEED_SLOW_R);
}


/*
*********************************************************************************************************
*	函 数 名: bsp_SearchTurnLeftFastBack
*	功能说明: 快速左转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchTurnLeftFastBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -TURN_LEFT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,-TURN_LEFT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SearchTurnLeftSlowBack
*	功能说明: 慢速左转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SearchTurnLeftSlowBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -TURN_LEFT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,-TURN_LEFT_SPEED_SLOW_R);
}


/*
*********************************************************************************************************
*	函 数 名: bsp_RotateCCW
*	功能说明: 原地旋转，左右轮都动，逆时针
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_RotateCCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, ROTATE_CCW_SPEED_L);
	bsp_SetMotorSpeed(MotorRight,ROTATE_CCW_SPEED_R);
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

uint32_t gIsTouchChargePile = 0 ;
uint32_t gIsCharging = 0 ;
uint32_t gIsChargeDone = 0 ;

	
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
		gIsTouchChargePile = 1 ;
		return true ;
	}
	else
	{
		gIsTouchChargePile = 0 ;
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
		gIsCharging = 1 ;
		return true ;
	}
	else
	{
		gIsCharging = 0 ;
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
		gIsChargeDone = 1 ;
		return true ;
	}
	else
	{
		gIsChargeDone = 0 ;
		return false ;
	}
}





