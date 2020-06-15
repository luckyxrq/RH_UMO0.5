#include "bsp.h"

typedef struct
{
	__IO bool isRunning;
	__IO uint32_t action;
	__IO uint32_t delay;
	
	float angle;
	uint32_t pulse;
}CCWRotationDrawArc;

static CCWRotationDrawArc ccwRotationDrawArc;  /*先逆时针旋转 再 画弧线*/

/*
*********************************************************************************************************
*	函 数 名: bsp_IsCCWRotationDrawArc
*	功能说明: 是否正在执行（先逆时针旋转 再 画弧线）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_IsCCWRotationDrawArc(void)
{
	return ccwRotationDrawArc.isRunning;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_CCWRotationDrawArcProc
*	功能说明: 先逆时针旋转 再 画弧线
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_CCWRotationDrawArcProc(void)
{
	if(!ccwRotationDrawArc.isRunning)
		return;
	
	switch(ccwRotationDrawArc.action)
	{
		case 0: /*逆时针旋转*/
		{
			ccwRotationDrawArc.angle = REAL_ANGLE();
			bsp_SetMotorSpeed(MotorLeft,-3);
			bsp_SetMotorSpeed(MotorRight,3);
			++ccwRotationDrawArc.action;
		}break;
		
		case 1: /*划曲线*/
		{
			if(ABS(REAL_ANGLE() - bsp_AngleAdd(ccwRotationDrawArc.angle, 90)) <= 10)
			{
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,2);
				
				/*记录开始划曲线的起始角度和时间，防止一直陷入死循环画曲线*/
				ccwRotationDrawArc.delay = xTaskGetTickCount();
				ccwRotationDrawArc.angle = REAL_ANGLE();
				++ccwRotationDrawArc.action;
			}
		}break;
		
		case 2:
		{
			/*如果没有满足下面的那个else if，而发生了碰撞，则认为在充电桩的另一边，需要反方向划曲线*/
			if(bsp_CollisionScan() != CollisionNone) 
			{
				ccwRotationDrawArc.pulse = bsp_GetCurrentBothPulse();
				bsp_SetMotorSpeed(MotorLeft, -3);
				bsp_SetMotorSpeed(MotorRight,-3);
				
				ccwRotationDrawArc.action = 3;
			}
			/*划曲线的过程中发现已经在充电桩附近则直接进入中间程序*/
			else if(INCLINATION_GO_L_0 || INCLINATION_GO_L_1 || INCLINATION_GO_L_2 || INCLINATION_GO_R_0 || INCLINATION_GO_R_1 || INCLINATION_GO_R_2 || ROTATE_CW || ROTATE_CCW) 
			{
				
			}
			/*划曲线几乎旋转了一圈，则退出这个模式*/
			else if(ABS(REAL_ANGLE() - bsp_AngleAdd(ccwRotationDrawArc.angle, 300)) <= 10 && (xTaskGetTickCount() - ccwRotationDrawArc.delay) >= 2000) 
			{
				
			}
		}break;
		
		case 3:
		{
			if(bsp_GetCurrentBothPulse() - ccwRotationDrawArc.pulse >= _SEARCH_PILE_GO_BACK_PULSE) /*通过探测得知，之前判断的方位是反的，所以顺时针旋转，再去划曲线*/
			{
				ccwRotationDrawArc.angle = REAL_ANGLE();
				bsp_SetMotorSpeed(MotorLeft, 3);
				bsp_SetMotorSpeed(MotorRight,-3);
				++ccwRotationDrawArc.action;
			}
		}break;
		
		case 4:
		{
			
		}break;
	}
}



