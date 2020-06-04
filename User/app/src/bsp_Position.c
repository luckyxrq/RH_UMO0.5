#include "bsp.h"
#include <math.h>

#define UPDATE_POS_T      20 /*���������ʱ�����ڣ���λMM*/
#define MAX_POSITION_XY  5

static Position position;
static int global_currentX,global_currentY;

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

void bsp_ResetPosArgument(void)
{
	
	/*��һʱ��λ����Ϣ*/
	position.lastX = 0;
	position.lastY = 0;
	/*��ǰʱ��λ����Ϣ*/
	position.currentX = 0;
	position.currentY = 0;
	/*��һʱ�̺͵�ǰ�ٶ�*/
	position.lastSpeed = 0;
	position.currentSpeed = 0;
//	/*��һʱ�̺͵�ǰʱ�̽Ƕ�*/
//	position.lastOrientation = 0;
//	position.currentOrientation = 0;
	
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
*	����˵��: ���ص�ǰX����,MM
*	��    ��: ��
*	�� �� ֵ: X���꣬mm
*********************************************************************************************************
*/
int32_t bsp_GetCurrentPosX(void)
{
	return (int)(position.currentX * 1000);
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_StartUpdatePos
*	����˵��: ���ص�ǰY����,MM
*	��    ��: ��
*	�� �� ֵ: Y���꣬mm
*********************************************************************************************************
*/
int32_t bsp_GetCurrentPosY(void)
{
	return (int)(position.currentY * 1000);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetCurrentOrientation
*	����˵��: ���ص�ǰ�����,rad
*	��    ��: ��
*	�� �� ֵ: ����ǣ�rad
*********************************************************************************************************
*/
double  bsp_GetCurrentOrientation(void)
{
	return position.currentOrientation;/*��λ������*/
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
			
			//��λ������ͼ���߽� λ����0 ��ͼ��0 ��ɨ״̬��0
			if(position.currentX > MAX_POSITION_XY || position.currentX < -MAX_POSITION_XY || position.currentY > MAX_POSITION_XY || position.currentY < -MAX_POSITION_XY) 
			{
				global_currentX += position.currentX;
				global_currentY += position.currentY;
				position.currentX = 0;
				position.currentY = 0;
				
				bsp_ResetCleanStrategyBStatus();
				bsp_StartUpdateGridMap();
			}
			 
			
		}break;
		
		case 2:
		{
			if(xTaskGetTickCount() - position.delay >= UPDATE_POS_T)
			{
				//bsp_SendReportFrame();/*�ϱ�Э��֡*/
				
				position.action = 1 ;
			}
		}break;
	}
	
 
}







