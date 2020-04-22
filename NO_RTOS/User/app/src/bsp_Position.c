#include "bsp.h"
#include <math.h>

#define UPDATE_POS_T      20 /*更新坐标的时间周期，单位MM*/

static Position position;


/*
*********************************************************************************************************
*	函 数 名: bsp_StartUpdatePos
*	功能说明: 开启周期性的更新坐标
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartUpdatePos(void)
{
	
	
	position.action = 0 ;
	position.delay = 0 ;
	position.isRunning = true;
}

void bsp_ResetPosArgument(void)
{
	
	/*上一时刻位置信息*/
	position.lastX = 0;
	position.lastY = 0;
	/*当前时刻位置信息*/
	position.currentX = 0;
	position.currentY = 0;
	/*上一时刻和当前速度*/
	position.lastSpeed = 0;
	position.currentSpeed = 0;
//	/*上一时刻和当前时刻角度*/
//	position.lastOrientation = 0;
//	position.currentOrientation = 0;
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StartUpdatePos
*	功能说明: 关闭周期性的更新坐标
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopUpdatePos(void)
{
	position.isRunning = false;
	position.action = 0 ;
	position.delay = 0 ;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_GetCurrentPosX
*	功能说明: 返回当前X坐标,MM
*	形    参: 无
*	返 回 值: X坐标，mm
*********************************************************************************************************
*/
int32_t bsp_GetCurrentPosX(void)
{
	return position.currentX * 1000;
}



/*
*********************************************************************************************************
*	函 数 名: bsp_StartUpdatePos
*	功能说明: 返回当前Y坐标,MM
*	形    参: 无
*	返 回 值: Y坐标，mm
*********************************************************************************************************
*/
int32_t bsp_GetCurrentPosY(void)
{
	return position.currentY * 1000;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetCurrentOrientation
*	功能说明: 返回当前航向角,rad
*	形    参: 无
*	返 回 值: 航向角，rad
*********************************************************************************************************
*/
double  bsp_GetCurrentOrientation(void)
{
	return position.currentOrientation;/*单位：弧度*/
}


/*
*********************************************************************************************************
*	函 数 名: bsp_PositionUpdate
*	功能说明: 周期性的更新坐标
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_PositionUpdate(void)
{
	if(!position.isRunning)
	{
		return ;
	}
	
	switch(position.action)
	{
		case 0:
		{
			position.currentTimestamp = bsp_GetRunTime();
			position.action++;
		};/*此处故意不加break，为了上上面那条语句只运行1次*/
		
		case 1:
		{
			/*保存上一次信息*/
			position.lastTimestamp = position.currentTimestamp;
			position.lastSpeed = position.currentSpeed;
			position.lastOrientation = position.currentOrientation;
			position.lastX = position.currentX;
			position.lastY = position.currentY;
			
			/*更新当前信息*/
			position.currentTimestamp = bsp_GetRunTime();
			position.currentSpeed = (bsp_MotorGetSpeed(MotorLeft) + bsp_MotorGetSpeed(MotorRight)) * 0.5F * 0.001F; /*单位：MM/S*/
			position.currentOrientation = Deg2Rad(bsp_AngleReadRaw()*0.01F); /*单位：弧度*/
			
			position.currentX = position.lastX + (position.currentSpeed*cos(position.currentOrientation)+position.lastSpeed*cos(position.lastOrientation))*(position.currentTimestamp-position.lastTimestamp)*0.0005F;
			position.currentY = position.lastY + (position.currentSpeed*sin(position.currentOrientation)+position.lastSpeed*sin(position.lastOrientation))*(position.currentTimestamp-position.lastTimestamp)*0.0005F;

			position.delay = bsp_GetRunTime(); /*确保周期性*/
			position.action++;
		}break;
		
		case 2:
		{
			if(bsp_GetRunTime() - position.delay >= UPDATE_POS_T)
			{
				//bsp_SendReportFrame();/*上报协议帧*/
				
				position.action = 1 ;
			}
		}break;
	}
	
 
}



