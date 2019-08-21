#include "bsp.h"
#include <math.h>

#define UPDATE_POS_T      20 /*���������ʱ�����ڣ���λMM*/

static Position position;


/*
*********************************************************************************************************
*	�� �� ��: bsp_StartUpdatePos
*	����˵��: ���������Եĸ�������
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: bsp_StartUpdatePos
*	����˵��: �ر������Եĸ�������
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: bsp_GetCurrentPosX
*	����˵��: ���ص�ǰX����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int32_t bsp_GetCurrentPosX(void)
{
	return position.currentX;
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_StartUpdatePos
*	����˵��: ���ص�ǰY����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int32_t bsp_GetCurrentPosY(void)
{
	return position.currentY;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_PositionUpdate
*	����˵��: �����Եĸ�������
*	��    ��: ��
*	�� �� ֵ: ��
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
		};/*�˴����ⲻ��break��Ϊ���������������ֻ����1��*/
		
		case 1:
		{
			/*������һ����Ϣ*/
			position.lastTimestamp = position.currentTimestamp;
			position.lastSpeed = position.currentSpeed;
			position.lastOrientation = position.currentOrientation;
			position.lastX = position.currentX;
			position.lastY = position.currentY;
			
			/*���µ�ǰ��Ϣ*/
			position.currentTimestamp = xTaskGetTickCount();
			position.currentSpeed = (bsp_MotorGetSpeed(MotorLeft) + bsp_MotorGetSpeed(MotorRight)) * 0.5F * 0.001F; /*��λ��MM/S*/
			position.currentOrientation = Deg2Rad(bsp_AngleReadRaw()*0.01F); /*��λ������*/
			
			position.currentX = position.lastX + (position.currentSpeed*cos(position.currentOrientation)+position.lastSpeed*cos(position.lastOrientation))*(position.currentTimestamp-position.lastTimestamp)*0.0005F;
			position.currentY = position.lastY + (position.currentSpeed*sin(position.currentOrientation)+position.lastSpeed*sin(position.lastOrientation))*(position.currentTimestamp-position.lastTimestamp)*0.0005F;

			position.delay = xTaskGetTickCount(); /*ȷ��������*/
			position.action++;
		}break;
		
		case 2:
		{
			if(xTaskGetTickCount() - position.delay >= UPDATE_POS_T)
			{
				bsp_SendReportFrame();/*�ϱ�Э��֡*/
				
				position.action = 1 ;
			}
		}break;
	}
	
 
}





/**************************************************α���룬2019��8��20��10:09:29****************************************

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

