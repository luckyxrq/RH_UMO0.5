#include "bsp.h"

#define CONSTANT	1.0F

#define NORMAL_SPEED	7000 /*�����ٶ�*/
#define SLOW_SPEED	    6000 /*���ٺ��ٶ�*/
#define INTERVAL        1000  /*ת���ٶȲ�*/

typedef struct
{
	volatile bool isRunning;
	volatile uint8_t action;
	volatile uint32_t delay;
	
}Edgewise;


static Edgewise edgewise;





void bsp_EdgewiseAct(void)
{
	float voltage;
	Collision collision;
	
//	if(!edgewise.isRunning)
//	{
//		return ;
//	}
	
	/*
		���ұ߿����ӵ͵����壬��ֱ���ر�
		����ұ�ײ���ˣ������ͬʱ����ת��תһ���̶��ĽǶ�
		������ʼ�����رߣ��Ӳ�������������ס����
	*/
	switch(edgewise.action)
	{
		case 0:/*�ж��Ƿ�����ر�ģʽ*/
		{
			voltage = bsp_GetInfraredVoltageRight();
			collision = bsp_CollisionScan();
			
			/*ײ���ˣ������жϣ�����ͬʱת��*/
			if(collision == CollisionRight)
			{
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				edgewise.action++ ;
			}
			/*����ߵ����壬���٣��ر�*/
			else if(voltage >= CONSTANT)
			{
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				edgewise.action = 10;
			}
			
		};break;
		
		case 1:/*ײ���ˣ������жϣ�����*/
		{
			bsp_SetMotorPWM(MotorLeft,Backward, SLOW_SPEED);
			bsp_SetMotorPWM(MotorRight,Backward,SLOW_SPEED);
			edgewise.delay = xTaskGetTickCount();
			edgewise.action++ ;
		};break;
		
		case 2:/*ײ���ˣ������жϣ�ԭ��ת��*/
		{
			if(xTaskGetTickCount() - edgewise.delay >= 600)
			{
				bsp_SetMotorPWM(MotorLeft,Backward, SLOW_SPEED);
				bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED);
				edgewise.delay = xTaskGetTickCount();
				edgewise.action++ ;
			}
		}break;
		
		case 3:/*ײ���ˣ������жϣ�����ǰ��*/
		{
			if(xTaskGetTickCount() - edgewise.delay >= 200)
			{
				bsp_SetMotorPWM(MotorLeft,Forward, SLOW_SPEED);
				bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED);
				edgewise.delay = xTaskGetTickCount();
				edgewise.action = 0 ;
			}
		}break;
		
		
		
		case 10:
		{
			collision = bsp_CollisionScan();
			
			/*ײ���ˣ������жϣ�����ͬʱת��*/
			if(collision == CollisionRight)
			{
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				edgewise.action = 1 ;
			}
			else
			{
				voltage = bsp_GetInfraredVoltageRight();
				
				if(voltage >= 1.2F)/*�׿����ˣ�����ƫ��*/
				{
					bsp_SetMotorPWM(MotorLeft,Forward, SLOW_SPEED);
					bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED+INTERVAL);
				}
				else if(voltage >= 1.1F) /*̫Զ�ˣ�����ƫ��*/
				{
					bsp_SetMotorPWM(MotorLeft,Forward, SLOW_SPEED);
					bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED+500);
				}
				else if(voltage <= 0.8F) /*̫Զ�ˣ�����ƫ��*/
				{
					bsp_SetMotorPWM(MotorLeft,Forward, SLOW_SPEED+500);
					bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED);
				}
				else if(voltage <= 0.9F) /*̫Զ�ˣ�����ƫ��*/
				{
					bsp_SetMotorPWM(MotorLeft,Forward, SLOW_SPEED+INTERVAL);
					bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED);
				}
				else
				{
					bsp_SetMotorPWM(MotorLeft,Forward, SLOW_SPEED);
					bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED);
				}
			}
		}break;
		
	}
}



