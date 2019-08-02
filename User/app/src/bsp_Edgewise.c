#include "bsp.h"

#define CONSTANT	1.0F

#define NORMAL_SPEED	7000 /*正常速度*/
#define SLOW_SPEED	    6000 /*减速后速度*/
#define INTERVAL        1000  /*转向速度差*/

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
		往右边靠，从低到波峰，则直接沿边
		如果右边撞到了，则后退同时向左转向，转一个固定的角度
		现在则开始进行沿边，从波峰的左边向右锁住波峰
	*/
	switch(edgewise.action)
	{
		case 0:/*判断是否进入沿边模式*/
		{
			voltage = bsp_GetInfraredVoltageRight();
			collision = bsp_CollisionScan();
			
			/*撞到了，优先判断，后退同时转向*/
			if(collision == CollisionRight)
			{
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				edgewise.action++ ;
			}
			/*从左边到波峰，减速，沿边*/
			else if(voltage >= CONSTANT)
			{
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				edgewise.action = 10;
			}
			
		};break;
		
		case 1:/*撞到了，优先判断，后退*/
		{
			bsp_SetMotorPWM(MotorLeft,Backward, SLOW_SPEED);
			bsp_SetMotorPWM(MotorRight,Backward,SLOW_SPEED);
			edgewise.delay = xTaskGetTickCount();
			edgewise.action++ ;
		};break;
		
		case 2:/*撞到了，优先判断，原地转向*/
		{
			if(xTaskGetTickCount() - edgewise.delay >= 600)
			{
				bsp_SetMotorPWM(MotorLeft,Backward, SLOW_SPEED);
				bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED);
				edgewise.delay = xTaskGetTickCount();
				edgewise.action++ ;
			}
		}break;
		
		case 3:/*撞到了，优先判断，继续前进*/
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
			
			/*撞到了，优先判断，后退同时转向*/
			if(collision == CollisionRight)
			{
				bsp_MotorBrake(MotorLeft);
				bsp_MotorBrake(MotorRight);
				edgewise.action = 1 ;
			}
			else
			{
				voltage = bsp_GetInfraredVoltageRight();
				
				if(voltage >= 1.2F)/*套靠近了，向左偏移*/
				{
					bsp_SetMotorPWM(MotorLeft,Forward, SLOW_SPEED);
					bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED+INTERVAL);
				}
				else if(voltage >= 1.1F) /*太远了，向右偏移*/
				{
					bsp_SetMotorPWM(MotorLeft,Forward, SLOW_SPEED);
					bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED+500);
				}
				else if(voltage <= 0.8F) /*太远了，向右偏移*/
				{
					bsp_SetMotorPWM(MotorLeft,Forward, SLOW_SPEED+500);
					bsp_SetMotorPWM(MotorRight,Forward,SLOW_SPEED);
				}
				else if(voltage <= 0.9F) /*太远了，向右偏移*/
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



