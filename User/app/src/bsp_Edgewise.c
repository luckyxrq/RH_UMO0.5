#include "bsp.h"

#define CONSTANT	1.0F
#define MIN_VOLTAGE	0.6F

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

extern uint8_t flgdec ;

void bsp_EdgewiseAct(void)
{
	Collision collision;
	

	switch(edgewise.action)
	{
		case 0:/*�ж��Ƿ�����ر�ģʽ*/
		{
			collision = bsp_CollisionScan();
			
			/*ײ���ˣ������жϣ�����ͬʱת��*/
			if(collision == CollisionLeft)
			{
				bsp_StopRunStable();
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				edgewise.action = 2 ;
			}
			/*ײ���ˣ������жϣ�����ͬʱת��*/
			else if(collision == CollisionRight)
			{
				bsp_StopRunStable();
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				edgewise.action = 3 ;
			}
			/*ײ���ˣ������жϣ�����ͬʱת��*/
			else if(collision == CollisionAll)
			{
				bsp_StopRunStable();
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				edgewise.action = 1 ;
			}
			
			
		};break;
		
		
		
		
		case 1:
		{
			bsp_SetMotorPWM(MotorLeft,Backward, SLOW_SPEED);
			bsp_SetMotorPWM(MotorRight,Backward,SLOW_SPEED);
			edgewise.delay = xTaskGetTickCount();
			edgewise.action = 10 ;
		}break;
		
		
		case 2:
		{
			bsp_SetMotorPWM(MotorLeft,Backward, SLOW_SPEED-500);
			bsp_SetMotorPWM(MotorRight,Backward,SLOW_SPEED+500);
			edgewise.delay = xTaskGetTickCount();
			edgewise.action = 10 ;
		}break;
		
		
		case 3:
		{
			bsp_SetMotorPWM(MotorLeft,Backward, SLOW_SPEED+500);
			bsp_SetMotorPWM(MotorRight,Backward,SLOW_SPEED-500);
			edgewise.delay = xTaskGetTickCount();
			edgewise.action = 10 ;
		}break;
		
		case 10:
		{
			if(xTaskGetTickCount() - edgewise.delay >= 1000)
			{
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				
				bsp_SetMotorPWM(MotorLeft,Forward, 7000);
				bsp_SetMotorPWM(MotorRight,Forward,7000);
				
				flgdec = 0 ;
				edgewise.action = 0 ;
			}
		}break;
		
		

	}	
}






