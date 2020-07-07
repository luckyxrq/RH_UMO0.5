#include "bsp.h"
#include <math.h>

#define UPDATE_POS_T      20 /*更新坐标的时间周期，单位MM*/
#define MAX_POSITION_XY  5

static Position position;
static int global_currentX,global_currentY;

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
	return (int)(position.currentX * 1000);
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
	return (int)(position.currentY * 1000);
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
			position.currentTimestamp = xTaskGetTickCount();
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
			position.currentTimestamp = xTaskGetTickCount();
			position.currentSpeed = (bsp_MotorGetSpeed(MotorLeft) + bsp_MotorGetSpeed(MotorRight)) * 0.5F * 0.001F; /*单位：MM/S*/
			position.currentOrientation = Deg2Rad(bsp_AngleReadRaw()*0.01F); /*单位：弧度*/
			
			position.currentX = position.lastX + (position.currentSpeed*cos(position.currentOrientation)+position.lastSpeed*cos(position.lastOrientation))*(position.currentTimestamp-position.lastTimestamp)*0.0005F;
			position.currentY = position.lastY + (position.currentSpeed*sin(position.currentOrientation)+position.lastSpeed*sin(position.lastOrientation))*(position.currentTimestamp-position.lastTimestamp)*0.0005F;

			position.delay = xTaskGetTickCount(); /*确保周期性*/
			position.action++;
			
			//定位超出地图最大边界 位置清0 地图清0 清扫状态清0
//			if(position.currentX > MAX_POSITION_XY || position.currentX < -MAX_POSITION_XY || position.currentY > MAX_POSITION_XY || position.currentY < -MAX_POSITION_XY) 
//			{
//				global_currentX += position.currentX;
//				global_currentY += position.currentY;
//				position.currentX = 0;
//				position.currentY = 0;
//				
//				bsp_ResetCleanStrategyBStatus();
//				bsp_StartUpdateGridMap();
//			}
			 
			
		}break;
		
		case 2:
		{
			if(xTaskGetTickCount() - position.delay >= UPDATE_POS_T)
			{
				//bsp_SendReportFrame();/*上报协议帧*/
				//bsp_SendReportFrameWithCRC16();
				
				position.action = 1 ;
			}
		}break;
	}
	
 
}





/**************************************************伪代码，2019年8月20日10:09:29****************************************

	int32_t leftCurrentSpeed  = 0 ;
	int32_t rightCurrentSpeed = 0 ;

	int32_t leftLastSpeed  = 0 ;
	int32_t rightLastSpeed = 0 ;

	double lastSpeed  = 0 ;
	double currentSpeed = 0 ;

	int32_t currentX = 0;
	int32_t currentY = 0;

	int32_t lastX = 0;
	int32_t lastY = 0;

	uint32_t currentTimestamp = 0;
	uint32_t lastTimestamp = 0;

	double last_orientation = 0;
	double current_orientation = 0;

	currentTimestamp = xTaskGetTickCount();
	{//per 20ms

		lastSpeed = currentSpeed;
		lastTimestamp = currentTimestamp; 
		last_orientation = current_orientation;
		lastX = currentX;
		lastY = currentY;

		currentTimestamp = xTaskGetTickCount();
		leftCurrentSpeed  = bsp_MotorGetSpeed(MotorLeft);
		rightCurrentSpeed = bsp_MotorGetSpeed(MotorRight);
		currentSpeed = (leftCurrentSpeed + rightCurrentSpeed) *0.5F*0.001 ; //m/s
		current_orientation = Deg2Rad(bsp_AngleReadRaw()*0.01f);



		currentX = lastX + (currentSpeed*cos(current_orientation)+lastSpeed*cos(last_orientation))*(currentTimestamp-lastTimestamp)*0.0005f;
		currentY = lastY + (currentSpeed*sin(current_orientation)+lastSpeed*sin(last_orientation))*(currentTimestamp-lastTimestamp)*0.0005f;

	}
	
	
****************************************************************************************************************/

