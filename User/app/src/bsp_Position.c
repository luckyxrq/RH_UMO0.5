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
*	功能说明: 返回当前X坐标
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int32_t bsp_GetCurrentPosX(void)
{
	return position.currentX;
}



/*
*********************************************************************************************************
*	函 数 名: bsp_StartUpdatePos
*	功能说明: 返回当前Y坐标
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int32_t bsp_GetCurrentPosY(void)
{
	return position.currentY;
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
		}break;
		
		case 2:
		{
			if(xTaskGetTickCount() - position.delay >= UPDATE_POS_T)
			{
				bsp_SendReportFrame();/*上报协议帧*/
				
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

