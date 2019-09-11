#include "bsp.h"

#define STRAIGHT_SPEED_FAST      3
#define STRAIGHT_SPEED_SLOW      3

#define TURN_RIGHT_SPEED_FAST_L  3
#define TURN_RIGHT_SPEED_FAST_R  1

//
#define TURN_RIGHT_SPEED_SLOW_L  4
#define TURN_RIGHT_SPEED_SLOW_R  2


#define TURN_LEFT_SPEED_FAST_L   1
#define TURN_LEFT_SPEED_FAST_R   3
                     
//					 
#define TURN_LEFT_SPEED_SLOW_L   2
#define TURN_LEFT_SPEED_SLOW_R   4

#define PIROUETTE_SPEED          1

#define ROTATE_CW_SPEED_L        2
#define ROTATE_CW_SPEED_R        -2

#define ROTATE_CCW_SPEED_L       -2
#define ROTATE_CCW_SPEED_R       2

#define BACKWARD_SPEED           -6


typedef struct
{
	volatile bool isRunning;
	uint32_t action;
	uint32_t delay;
	
	
}EdgewiseRun;

static EdgewiseRun edgewiseRun;
static void bsp_EdgewiseRunStraightFast(void);
static void bsp_EdgewiseRunStraightSlow(void);
static void bsp_EdgewiseTurnRightFast(void)  ;
static void bsp_EdgewiseTurnRightSlow(void)  ;
static void bsp_EdgewiseTurnLeftFast(void)   ;
static void bsp_EdgewiseTurnLeftSlow(void)   ;
static void bsp_PirouetteCW(void)            ;
static void bsp_PirouetteCCW(void)           ;
static void bsp_RotateCW(void)               ;
static void bsp_RotateCCW(void)              ;
static void bsp_GoBackward(void)             ;

/*
*********************************************************************************************************
*	函 数 名: bsp_StartEdgewiseRun
*	功能说明: 开启沿边行走
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartEdgewiseRun(void)
{
	edgewiseRun.action = 0 ;
	edgewiseRun.delay = 0 ;
	edgewiseRun.isRunning = true;
	
	/*消除编译器警告*/
	UNUSED(bsp_EdgewiseRunStraightFast) ;
	UNUSED(bsp_EdgewiseRunStraightSlow) ;
	UNUSED(bsp_EdgewiseTurnRightFast)   ;
	UNUSED(bsp_EdgewiseTurnRightSlow)   ;
	UNUSED(bsp_EdgewiseTurnLeftFast)    ;
	UNUSED(bsp_EdgewiseTurnLeftSlow)    ;
	UNUSED(bsp_PirouetteCW)             ;
	UNUSED(bsp_PirouetteCCW)            ;
	UNUSED(bsp_GoBackward)              ;
	UNUSED(bsp_RotateCW)                ;
	UNUSED(bsp_RotateCCW)               ;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StopEdgewiseRun
*	功能说明: 关闭沿边行走
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopEdgewiseRun(void)
{
	edgewiseRun.isRunning = false;
	edgewiseRun.action = 0 ;
	edgewiseRun.delay = 0 ;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitEdgewiseRun
*	功能说明: 初始化沿边行走
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_EdgewiseRun(void)
{
	if(!edgewiseRun.isRunning)
		return ;
	
	switch(edgewiseRun.action)
	{
		case 0:/*进入沿边模式，首先直走*/
		{
			bsp_EdgewiseRunStraightSlow();
			edgewiseRun.action++;
		}break;
		
		case 1:/*如果发生了碰撞，碰撞后退*/
		{
			Collision ret = bsp_CollisionScan();
			float vol = bsp_GetInfraredVoltageRight();
				
			if(ret != CollisionNone)
			{
				bsp_GoBackward();
				edgewiseRun.delay = xTaskGetTickCount();
				edgewiseRun.action++;
			}
			else if(vol >= 1.2F && vol <=2.5F )
			{
				bsp_EdgewiseRunStraightSlow();
			}
			else if(vol < 1.2F)
			{
				bsp_EdgewiseTurnRightSlow();
			}
			else if(vol > 2.5F)
			{
				bsp_EdgewiseTurnLeftSlow();
			}
		}break;
		
		case 2:/*后退完了再原地旋转，左右都动*/
		{
			if(xTaskGetTickCount() - edgewiseRun.delay >= 800)
			{
				bsp_RotateCCW();
				edgewiseRun.delay = xTaskGetTickCount();
				edgewiseRun.action++;
			}
		}break;
		
		case 3:/*旋转一会儿，继续直行，回到状态1*/
		{
			if(xTaskGetTickCount() - edgewiseRun.delay >= 800)
			{
				bsp_EdgewiseRunStraightSlow();
				edgewiseRun.action = 1;
			}
		}break;

		
		
	}
}


/*
*********************************************************************************************************
*	函 数 名: bsp_EdgewiseRunStraightFast
*	功能说明: 快速直行
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_EdgewiseRunStraightFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, STRAIGHT_SPEED_FAST);
	bsp_SetMotorSpeed(MotorRight,STRAIGHT_SPEED_FAST);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_EdgewiseRunStraightSlow
*	功能说明: 慢速直行
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_EdgewiseRunStraightSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, STRAIGHT_SPEED_SLOW);
	bsp_SetMotorSpeed(MotorRight,STRAIGHT_SPEED_SLOW);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_EdgewiseTurnRightFast
*	功能说明: 快速右转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_EdgewiseTurnRightFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_RIGHT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,TURN_RIGHT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_EdgewiseTurnRightSlow
*	功能说明: 慢速右转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_EdgewiseTurnRightSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_RIGHT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,TURN_RIGHT_SPEED_SLOW_R);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_EdgewiseTurnLeftFast
*	功能说明: 快速左转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_EdgewiseTurnLeftFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_LEFT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,TURN_LEFT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_EdgewiseTurnLeftSlow
*	功能说明: 慢速左转
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_EdgewiseTurnLeftSlow(void)
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
*	函 数 名: bsp_RotateCW
*	功能说明: 原地旋转，左右轮都动，顺时针
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_RotateCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, ROTATE_CW_SPEED_L);
	bsp_SetMotorSpeed(MotorRight,ROTATE_CW_SPEED_R);
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
