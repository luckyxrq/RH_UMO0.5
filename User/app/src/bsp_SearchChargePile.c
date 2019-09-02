#include "bsp.h"

#define STRAIGHT_SPEED_FAST      2
#define STRAIGHT_SPEED_SLOW      1

#define TURN_RIGHT_SPEED_FAST_L  3
#define TURN_RIGHT_SPEED_FAST_R  1

#define TURN_RIGHT_SPEED_SLOW_L  2
#define TURN_RIGHT_SPEED_SLOW_R  1


#define TURN_LEFT_SPEED_FAST_L   1
#define TURN_LEFT_SPEED_FAST_R   3
                                 
#define TURN_LEFT_SPEED_SLOW_L   1
#define TURN_LEFT_SPEED_SLOW_R   2


typedef struct
{
	volatile bool isRunning;
	volatile uint32_t acion;
	volatile uint32_t delay;
		
}Serach;

static Serach search;
static void bsp_SearchRunStraightFast(void);
static void bsp_SearchRunStraightSlow(void);
static void bsp_SearchTurnRightFast(void)  ;
static void bsp_SearchTurnRightSlow(void)  ;
static void bsp_SearchTurnLeftFast(void)   ;
static void bsp_SearchTurnLeftSlow(void)   ;

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
	search.acion = 0 ;
	search.delay = 0 ;
	search.isRunning = true;
	
	/*防止编译器警告*/
	UNUSED(bsp_SearchRunStraightFast);
	UNUSED(bsp_SearchRunStraightSlow);
	UNUSED(bsp_SearchTurnRightFast)  ;
	UNUSED(bsp_SearchTurnRightSlow)  ;
	UNUSED(bsp_SearchTurnLeftFast)   ;
	UNUSED(bsp_SearchTurnLeftSlow)   ;
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
	search.acion = 0 ;
	search.delay = 0 ;
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
		return;
	
	switch(search.acion)
	{
		case 0:
		{
			bsp_SearchRunStraightSlow();
			search.acion++;
		}break;
		
		case 1:
		{
			if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)
				&& bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
			{
				bsp_SearchRunStraightSlow();
			}
			else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SearchTurnLeftSlow();
			}
			else if((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SearchTurnRightSlow();
			}
			
			search.acion++;
		}break;
		
		case 2:
		{
			search.acion = 1 ;
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


