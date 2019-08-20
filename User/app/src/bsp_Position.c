#include "bsp.h"
#include <math.h>

typedef struct
{
	/*��һʱ��λ����Ϣ*/
	volatile int32_t lastX ;
	volatile int32_t lastY ;
	
	/*��ǰʱ��λ����Ϣ*/
	int32_t currentX ;
	int32_t currentY ;
	
	/*��һʱ�̺͵�ǰ�ٶ�*/
	volatile double lastSpeed;
	volatile double currentSpeed;
	
	/*��һʱ�̺͵�ǰʱ��ʱ���*/
	volatile uint32_t lastTimestamp;
	volatile uint32_t currentTimestamp;
	
	/*��һʱ�̺͵�ǰʱ�̽Ƕ�*/
	volatile double lastOrientation ;
	volatile double currentOrientation ;
	
	/*״̬��*/
	volatile uint8_t action ;
	volatile bool isRunning ;
	volatile uint32_t delay ;
}Position;


static Position position;

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
		}break;
		
		case 1:
		{
			/*������һ����Ϣ*/
			position.lastSpeed = position.currentSpeed;
			position.lastTimestamp = position.currentTimestamp;
			position.lastOrientation = position.currentOrientation;
			position.lastX = position.currentX;
			position.lastY = position.currentY;
			
			/*���µ�ǰ��Ϣ*/
			position.currentTimestamp = xTaskGetTickCount();
			position.currentSpeed = (bsp_MotorGetSpeed(MotorLeft) + bsp_MotorGetSpeed(MotorRight)) * 0.5F * 0.001; /*��λ��MM/S*/
			position.currentOrientation = Deg2Rad(bsp_AngleReadRaw()*0.01F); /*��λ������*/
			
			position.currentX = position.lastX + (position.currentSpeed*cos(position.currentOrientation)+position.lastSpeed*cos(position.lastOrientation))*(position.currentTimestamp-position.lastTimestamp)*0.0005F;
			position.currentY = position.lastY + (position.currentSpeed*sin(position.currentOrientation)+position.lastSpeed*sin(position.lastOrientation))*(position.currentTimestamp-position.lastTimestamp)*0.0005F;

		}break;
	}
	
 
}





/**************************************************α����********************************************************

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

