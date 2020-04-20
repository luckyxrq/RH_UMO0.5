#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOB
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_3

#define GPIO_PORT_CHARGE_IS_DONE      GPIOB
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_4


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

#define MAX_SEARCH_TICK      (1000*120)

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
	volatile SearchChargePileCollision collision;
	
	volatile bool isOnChargePile;
	volatile uint32_t disconnectTimes;
	
	uint32_t startTick;
}Serach;


static Serach search;
static void bsp_InitIO(void);
static bool bsp_IsCharging(void);
static bool bsp_IsChargeDone(void);
static bool bsp_IsTouchChargePile(void);
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
	bsp_InitIO();
	
	search.action = 0 ;
	search.delay = 0 ;
	search.startTick = xTaskGetTickCount();
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
	search.action = 0 ;
	search.delay = 0 ;
	
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
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
	
	if(!search.isRunning)
	{
		if(search.isOnChargePile) /*之前别标记为正在充电状态了，但是现在没有上桩反馈，可能是脱落了，可能是充电桩正在PWM充电*/
		{
			
			if(bsp_IsChargeDone()) /*充满*/
			{
				bsp_LedOn(LED_LOGO_CLEAN);
				bsp_LedOn(LED_LOGO_POWER);
				bsp_LedOff(LED_LOGO_CHARGE);
				bsp_LedOff(LED_COLOR_YELLOW);
				bsp_LedOn(LED_COLOR_GREEN);
				bsp_LedOff(LED_COLOR_RED);
			}
			else if(bsp_IsCharging()) /*充电中*/
			{
				bsp_LedOn(LED_LOGO_CLEAN);
				bsp_LedOn(LED_LOGO_POWER);
				bsp_LedOff(LED_LOGO_CHARGE);
				bsp_LedOn(LED_COLOR_YELLOW);
				bsp_LedOff(LED_COLOR_GREEN);
				bsp_LedOff(LED_COLOR_RED);
			}
			else
			{
				bsp_LedOn(LED_LOGO_CLEAN);
				bsp_LedOn(LED_LOGO_POWER);
				bsp_LedOn(LED_LOGO_CHARGE);
				bsp_LedOff(LED_COLOR_YELLOW);
				bsp_LedOff(LED_COLOR_GREEN);
				bsp_LedOff(LED_COLOR_RED);
			}
			
			
			/*检测是不是挪开了*/
			if(!bsp_IsTouchChargePile())
			{
				if(++search.disconnectTimes >= 300)
				{
					search.isOnChargePile = false;
					bsp_SetKeyRunLastState(RUN_STATE_DEFAULT);
					
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}
			}
			else
			{
				search.disconnectTimes = 0 ;
			}
		}
		else /*之前没有被标记为已经上桩状态，但是现在有上涨反馈，应该是人为抱上去了*/
		{
			if(bsp_IsTouchChargePile() == true)
			{
				DEBUG("is charging...\r\n");
				bsp_SetMotorSpeed(MotorLeft,0);
				bsp_SetMotorSpeed(MotorRight,0);
				bsp_StopSearchChargePile();
				
				bsp_StopRunToggleLED();
				
				bsp_LedOn(LED_LOGO_CLEAN);
				bsp_LedOn(LED_LOGO_POWER);
				bsp_LedOff(LED_LOGO_CHARGE);
				bsp_LedOn(LED_COLOR_YELLOW);
				bsp_LedOff(LED_COLOR_GREEN);
				bsp_LedOff(LED_COLOR_RED);
				bsp_SperkerPlay(Song22);
				
				
				search.isOnChargePile = true;
			}
		}
		
		return;
	}
		
	/*计时，时间到了还没找到充电桩*/
	if(xTaskGetTickCount() - search.startTick >= MAX_SEARCH_TICK)
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_StopSearchChargePile();
		
		bsp_StopRunToggleLED();
		
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOn(LED_LOGO_CHARGE);
		bsp_LedOff(LED_COLOR_YELLOW);
		bsp_LedOff(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
		bsp_SperkerPlay(Song24);
		
		return;
	}
	
	
	
	/*充电*/
	if(bsp_IsTouchChargePile() == true)
	{
		DEBUG("is charging...\r\n");
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_StopSearchChargePile();
		
		bsp_StopRunToggleLED();
		
		bsp_LedOn(LED_LOGO_CLEAN);
		bsp_LedOn(LED_LOGO_POWER);
		bsp_LedOff(LED_LOGO_CHARGE);
		bsp_LedOn(LED_COLOR_YELLOW);
		bsp_LedOff(LED_COLOR_GREEN);
		bsp_LedOff(LED_COLOR_RED);
		bsp_SperkerPlay(Song22);
		
		
		search.isOnChargePile = true;
		
		return ;
	}
	
	switch(search.action)
	{
		case 0:
		{
			bsp_SearchRunStraightSlow();
			search.action++;
		}break;
		
		case 1:
		{
			/*首先判断碰撞*/
			Collision ret = bsp_CollisionScan();
			
			if(ret != CollisionNone)
			{
				/*不管如何碰到了就后退，在后退的过程中再来调节轮子*/
				bsp_GoBackward();
				
//				if((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)) ||
//					(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
//				{
//					search.collision = eHasSignalCollision;
//				}
//				else
//				{
//					search.collision = eNoSignalCollision;
//				}
				
				search.delay = xTaskGetTickCount();
				search.action++;
			}
			/*前面2个，都能收到左右发射*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)
				&& bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
			{
				bsp_SearchRunStraightSlow();
			}
			/*前面2个，各收各*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SearchRunStraightSlow();
			}
			
			
			/*1号能收到2个 ,2号能收到左边*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SearchRunStraightSlow();
			}
			
			/*2号能收到2个 ,1号能收到右边*/
			else if(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
			{
				bsp_SearchRunStraightSlow();
			}
			
			
			/*1，2号都能收到左边，都不能收到右边*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && 
				!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
			{
				bsp_SearchTurnRightSlow();
			}
			/*1，2号都能收到右边，都不能收到左边*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && 
				!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SearchTurnLeftSlow();
			}

			
			/*1号不能同时收到左右发射，2能同时收到左右发射*/
			else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SearchTurnLeftSlow();
			}
			/*1号能同时收到左右发射，2不能同时收到左右发射*/
			else if((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SearchTurnRightSlow();
			}
			/*侧面4号能收到广角和左或右，正面1,2哈不能收到任何,原地旋转*/
			else if(bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)))
			{
				bsp_PirouetteCW();
			}
			/*侧面3号能收到广角和左或右，正面1,2哈不能收到任何,原地旋转*/
			else if(bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT)))
			{
				bsp_PirouetteCCW();
			}
			/*1，2都不能同时收到左右发射*/
			else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SearchRunStraightSlow();
			}
			
			
		}break;
		
		case 2:
		{
			/*充电最后一步撞上了*/
			if(xTaskGetTickCount() - search.delay >= 5000)
			{
				search.action = 1 ;
				
				if(!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH1,IR_TX_SITE_CENTER)
				&& !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_CENTER))
				{
					bsp_RotateCCW();
					search.delay = xTaskGetTickCount();
					search.action  = 3;
				}
				
				
			}
			else
			{
				/*前面2个，都能收到左右发射*/
				if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)
				&& bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
				{
					bsp_SearchRunStraightSlow();
					search.action = 1 ;
				}
				/*前面2个，各收各*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SearchRunStraightSlow();
					search.action = 1 ;
				}
				
				
				/*1号能收到2个 ,2号能收到左边*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SearchRunStraightSlowBack();
				}
				
				/*2号能收到2个 ,1号能收到右边*/
				else if(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				{
					bsp_SearchRunStraightSlowBack();
				}
				
				
				/*1，2号都能收到左边，都不能收到右边*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && 
					!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
				{
					bsp_SearchTurnLeftSlowBack();
				}
				/*1，2号都能收到右边，都不能收到左边*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && 
					!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SearchTurnRightSlowBack();
				}

				
				/*1号不能同时收到左右发射，2能同时收到左右发射*/
				else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SearchTurnRightSlowBack();
				}
				/*1号能同时收到左右发射，2不能同时收到左右发射*/
				else if((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SearchTurnLeftSlowBack();
				}
				
				
				/*侧面4号能收到广角和左或右，正面1,2哈不能收到任何,原地旋转*/
				else if(bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)))
				{
					bsp_PirouetteCW();
				}
				/*侧面3号能收到广角和左或右，正面1,2哈不能收到任何,原地旋转*/
				else if(bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT)))
				{
					bsp_PirouetteCCW();
				}
				/*1，2都不能同时收到左右发射*/
				else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SearchRunStraightSlowBack();
				}
			}
		}break;
		
		
		case 3:/*退完没有红外信号则转弯*/
		{
			if(xTaskGetTickCount() - search.delay >= 800)
			{
				search.action = 1 ;
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
static void bsp_InitIO(void)
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
static bool bsp_IsTouchChargePile(void)
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
static bool bsp_IsCharging(void)
{
	if(GPIO_ReadInputDataBit(GPIO_PORT_CHARGE_IS_CHARGING,GPIO_PIN_CHARGE_IS_CHARGING) == 0)
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
static bool bsp_IsChargeDone(void)
{
	if(GPIO_ReadInputDataBit(GPIO_PORT_CHARGE_IS_DONE,GPIO_PIN_CHARGE_IS_DONE) == 0)
	{
		return true ;
	}
	else
	{
		return false ;
	}
}

