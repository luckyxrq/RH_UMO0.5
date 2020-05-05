#include "bsp.h"

#define STRAIGHT_SPEED_FAST      12
#define STRAIGHT_SPEED_SLOW      6

#define TURN_RIGHT_SPEED_FAST_L  3
#define TURN_RIGHT_SPEED_FAST_R  1

//
#define TURN_RIGHT_SPEED_SLOW_L  6
#define TURN_RIGHT_SPEED_SLOW_R  5


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
#define POSSIBLE_END      66

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
static float myabs(float val)                ;
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
			if(bsp_GetInfraRedAdcVoltage(IR7) >= 1.0F )
			{
				edgewiseRun.ErlangGodStartTime = xTaskGetTickCount() ;
				bsp_EdgewiseRunStraightSlow();
			}
			else
			{
				if(xTaskGetTickCount() - edgewiseRun.ErlangGodStartTime >= 1000)
				{
					bsp_EdgewiseRunStraightFast();
				}
			}
			if(bsp_CollisionScan() != CollisionNone)
			{
				edgewiseRun.action++;
			}
		}break;
		
		case 1:/*如果发生了碰撞，碰撞后退*/
		{
			float vol = bsp_GetInfraredVoltageRight();
			edgewiseRun.collision = bsp_CollisionScan();
			
			if(edgewiseRun.collision != CollisionNone)
			{
				bsp_GoBackward();
				/*记录下当前的脉冲，用于退后指定脉冲数（距离），同时记录下当前时间，放置退了很久还没知道*/
				edgewiseRun.pulse = bsp_GetCurrentBothPulse();
				edgewiseRun.delay = xTaskGetTickCount();
				edgewiseRun.action++;
			}
			else if(vol >= 1.5F && vol <=2.0F )
			{
				bsp_EdgewiseRunStraightSlow();
				edgewiseRun.possibleEnd = 0 ;
			}
			/*向右靠近的过程中还需要检测靠近的时间，如果靠近了很久还是没能找到电压值，那么就是走到了尽头*/
			else if(vol < 1.5F)
			{
				bsp_EdgewiseTurnRightSlow();
				if(vol < 0.2F)
				{
					
					if(edgewiseRun.possibleEnd++ >= POSSIBLE_END)
					{
						edgewiseRun.possibleEnd = 0 ;
						edgewiseRun.action = 4 ;
					}
				}
			}
			else if(vol > 2.0F)
			{
				bsp_EdgewiseTurnLeftSlow();
				edgewiseRun.possibleEnd = 0 ;
			}

		}break;
		
		case 2:/*后退完了再原地旋转，左右都动*/
		{
			if((xTaskGetTickCount() - edgewiseRun.delay >= 2000) || (bsp_GetCurrentBothPulse()-edgewiseRun.pulse)>=GO_BACK_PULSE)
			{
				bsp_RotateCCW();
				/*获取角度和时间，转动固定角度，时间太久还没转到代表异常*/
				edgewiseRun.angle = bsp_AngleRead();
				edgewiseRun.delay = xTaskGetTickCount();
				edgewiseRun.action++;
			}
		}break;
		
		case 3:/*旋转一会儿，继续直行，回到状态1*/
		{
			float val = bsp_GetInfraredVoltageRight();
			if(myabs(bsp_AngleAdd(edgewiseRun.angle ,20) - (bsp_AngleRead())) <= 2.0F ||
				(val>=1.0F && val<=3.3F && myabs(bsp_AngleRead()-edgewiseRun.angle)>=10.0F ))
			{
				bsp_EdgewiseRunStraightSlow();
				edgewiseRun.action = 1;
			}

		}break;

		case 4: /*完全丢失，画大弧线，最大旋转角度*/
		{
			edgewiseRun.angle = bsp_AngleRead();
			edgewiseRun.delay = xTaskGetTickCount();
			
			
			bsp_PirouetteCW();
			edgewiseRun.action++;
		}break;
		
		case 5:
		{
			float vol = bsp_GetInfraredVoltageRight();
			
			
			/*判断下旋转了太久了*/
			if((xTaskGetTickCount() - edgewiseRun.delay)>= 3000 && 
				myabs(bsp_AngleAdd(edgewiseRun.angle ,360) - (bsp_AngleRead())) <= 10.0F)
			{
				edgewiseRun.action = 0 ;
			}
			

			if(bsp_CollisionScan()!=CollisionNone || (vol >= 1.2F && vol <=3.3F ))
			{
				edgewiseRun.action = 1 ;
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
#if 1
	
	bsp_SetMotorSpeed(MotorLeft, ROTATE_CCW_SPEED_L);
	bsp_SetMotorSpeed(MotorRight,ROTATE_CCW_SPEED_R);

#else
	
/*
	linearVelocity   线速度（毫米/s）
	angularVelocity  角速度（度/s）
	r                后退距离，也是转弯半径（毫米）
*/
	
	/*基本参数，计算合理的线速度和角速度*/
	double r = 15;
	double linearVelocity = 20;
	double angularVelocity = Deg2Rad(linearVelocity / r);

	/*计算出速度，单位MM/S */
	int16_t leftVelocity = (int16_t)((0.5*(2*linearVelocity*0.001 - Deg2Rad(angularVelocity)*WHEEL_LENGTH))* 1000);
	int16_t rightVelocity = (int16_t)((0.5*(2*linearVelocity*0.001 + Deg2Rad(angularVelocity)*WHEEL_LENGTH))* 1000);
	
	/*设置速度，单位MM/S */
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(leftVelocity));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(rightVelocity));
	
#endif

}



static float myabs(float val)
{
	if(val < 0)
	{
		val = - val;
	}
	
	return val;
}


