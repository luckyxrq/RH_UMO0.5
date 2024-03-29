#include "bsp.h"

#define STRAIGHT_SPEED_FAST      10
#define STRAIGHT_SPEED_SLOW      8

#define TURN_RIGHT_SPEED_FAST_L  3
#define TURN_RIGHT_SPEED_FAST_R  1

//
#define TURN_RIGHT_SPEED_SLOW_L  6
#define TURN_RIGHT_SPEED_SLOW_R  3


#define TURN_LEFT_SPEED_FAST_L   1
#define TURN_LEFT_SPEED_FAST_R   3
                     
//					 
#define TURN_LEFT_SPEED_SLOW_L         5
#define TURN_LEFT_SPEED_SLOW_R         6
                                       
#define PIROUETTE_SPEED                6
                                       
#define ROTATE_CW_SPEED_L              5
#define ROTATE_CW_SPEED_R              -5
                                       
#define ROTATE_CCW_SPEED_L             -5
#define ROTATE_CCW_SPEED_R             5
                                       
#define BACKWARD_SPEED                 -6
                                       
/*轮子后退20MM，脉冲数*/               
#define GO_BACK_PULSE                  (10/(3.14F*70)*1024)
#define COLLISION_STEERING_ANGLE       30.0F

/*向右守多少次认为出界*/
#define POSSIBLE_END      120

typedef struct
{
	volatile bool isRunning;
	uint32_t action;
	uint32_t delay;
	
	uint32_t pulse;
	float angle;
	Collision collision;
	uint32_t possibleEnd;
	uint32_t ErlangGodStartTime ;
	uint8_t Left_Right;
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
	edgewiseRun.collision = CollisionNone;
	edgewiseRun.possibleEnd = 0 ;
	edgewiseRun.ErlangGodStartTime = 0 ;
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
	edgewiseRun.collision = CollisionNone;
	edgewiseRun.possibleEnd = 0 ;
	edgewiseRun.ErlangGodStartTime = 0 ;

}

void bsp_SetEdgeLeftRight(Dir_Right_Left Edg_dir)
{
	if(Edg_dir  == Dir_left)
	{
		edgewiseRun.Left_Right = Dir_left;	
	}else if(Edg_dir  == Dir_right)
	{
		edgewiseRun.Left_Right = Dir_right;
	}
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
	static unsigned char IRSensorData[10] = {0};
	
	if(!edgewiseRun.isRunning)
		return ;
	
	bsp_GetAllIrIsObstacle(IRSensorData);
	
	switch(edgewiseRun.action)
	{
		case 0:/*进入沿边模式，首先直走*/
		{
			/* 传感器有数据  慢速*/
			if( IRSensorData[2] == 1 || IRSensorData[7] == 1 || \
			    IRSensorData[0] == 1 || IRSensorData[4] == 1 || \
			    IRSensorData[6] == 1)
			{
				edgewiseRun.ErlangGodStartTime = xTaskGetTickCount() ;
				bsp_EdgewiseRunStraightSlow();
			}
			/* 传感器长时间没数据  快速*/
			else
			{
				if(xTaskGetTickCount() - edgewiseRun.ErlangGodStartTime >= 1000)
				{
					bsp_EdgewiseRunStraightFast();
				}
			}
			/* 有碰撞 或者悬崖 进入下一个*/
			if(bsp_CollisionScan() != CollisionNone  \
				||  bsp_CliffIsDangerous(CliffLeft)|| bsp_CliffIsDangerous(CliffMiddle) || bsp_CliffIsDangerous(CliffRight))
			{
				edgewiseRun.action++;
			}
		}break;
		
		case 1:/*如果发生了碰撞，碰撞后退*/
		{
			float vol= 0 ;
			
			if(edgewiseRun.Left_Right == Dir_right ) vol = bsp_GetInfraredVoltageRight();
			if(edgewiseRun.Left_Right == Dir_left ) vol = bsp_GetInfraredVoltageLeft();
			
			edgewiseRun.collision = bsp_CollisionScan();
			if(bsp_CliffIsDangerous(CliffLeft)|| bsp_CliffIsDangerous(CliffMiddle) || bsp_CliffIsDangerous(CliffRight))
			{
				edgewiseRun.collision = CollisionAll;
			}
			
			/* 如果上一步发生了碰撞或者悬崖 这一步依旧存在 ， 进入下一步*/
			if(edgewiseRun.collision != CollisionNone)
			{
				bsp_GoBackward();
				/*记录下当前的脉冲，用于退后指定脉冲数（距离），同时记录下当前时间，放置退了很久还没知道*/
				edgewiseRun.pulse = bsp_GetCurrentBothPulse();
				edgewiseRun.delay = xTaskGetTickCount();
				edgewiseRun.action++;
			}
			/*向右靠近的过程中还需要检测靠近的时间，如果靠近了很久还是没能找到电压值，那么就是走到了尽头*/
			else if(vol < 1000)
			{
				if(edgewiseRun.Left_Right == Dir_right ) bsp_EdgewiseTurnRightSlow();
				if(edgewiseRun.Left_Right == Dir_left ) bsp_EdgewiseTurnLeftSlow();
				
				if(vol < 950)
				{
					
					if(edgewiseRun.possibleEnd++ >= POSSIBLE_END)
					{
						edgewiseRun.possibleEnd = 0 ;
						edgewiseRun.action = 4 ;
					}
				}
			}
			else if(vol > 1000)
			{
				if(edgewiseRun.Left_Right == Dir_right ) bsp_EdgewiseTurnLeftSlow();
				if(edgewiseRun.Left_Right == Dir_left ) bsp_EdgewiseTurnRightSlow();
				edgewiseRun.possibleEnd = 0 ;
			}

		}break;
		
		case 2:/*后退完了再原地旋转，左右都动*/
		{
			if((xTaskGetTickCount() - edgewiseRun.delay >= 2000) || (bsp_GetCurrentBothPulse()-edgewiseRun.pulse)>=GO_BACK_PULSE)
			{
				if(edgewiseRun.Left_Right == Dir_right ) bsp_RotateCCW();
				if(edgewiseRun.Left_Right == Dir_left ) bsp_RotateCW();
				
				/*获取脉冲数，时间太久还没转到代表异常*/
				edgewiseRun.pulse = bsp_GetCurrentBothPulse();

				edgewiseRun.action++;
			}
		}break;
		
		case 3:/*旋转一会儿，继续直行，回到状态1*/
		{
			if(bsp_GetCurrentBothPulse()-edgewiseRun.pulse >= CCW_30_PULSE)
			{
				bsp_EdgewiseRunStraightSlow();
				edgewiseRun.action = 1 ;
			}
		}break;

		case 4: /*完全丢失，画大弧线，最大旋转角度*/
		{
			edgewiseRun.pulse = bsp_GetCurrentBothPulse();
			
			if(edgewiseRun.Left_Right == Dir_right )  bsp_PirouetteCW();
			if(edgewiseRun.Left_Right == Dir_left ) bsp_PirouetteCCW();
			
			edgewiseRun.action++;
		}break;
		
		case 5:
		{
			float vol = 0;
			
			/*如果发生了碰撞  或者沿边IR值到了范围就开始沿边  回到1 */
			if(edgewiseRun.Left_Right == Dir_right ) vol = bsp_GetInfraredVoltageRight();
			if(edgewiseRun.Left_Right == Dir_left ) vol = bsp_GetInfraredVoltageLeft();
			
			edgewiseRun.collision = bsp_CollisionScan();
			if(bsp_CliffIsDangerous(CliffLeft)|| bsp_CliffIsDangerous(CliffMiddle) || bsp_CliffIsDangerous(CliffRight))
			{
				edgewiseRun.collision = CollisionAll;
			}
			if( edgewiseRun.collision !=CollisionNone || (vol >= 500 ))
			{
				edgewiseRun.action = 1 ;
			}
			
			
			/*如果转了一圈还没有墙边 则回到第一步 */
			if(bsp_GetCurrentBothPulse()-edgewiseRun.pulse >= CCW_360_PULSE)
			{
				bsp_EdgewiseRunStraightSlow();
				edgewiseRun.action = 0 ;
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




