#include "bsp.h"

#define CONSTANT	1.0F
#define MIN_VOLTAGE	0.6F

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
	//Collision collision;
	

	switch(edgewise.action)
	{
		case 0:
		{
			/*右边沿边红外检测到了*/
			if(bsp_GetInfraredVoltageRight() >= 1.0F)
			{
				edgewise.action++;
			}
		}break;
		
		case 1:
		{
			if(bsp_GetInfraredVoltageRight() >= 1.0F)
			{
				bsp_SetMotorSpeed(MotorLeft,5);
				bsp_SetMotorSpeed(MotorRight,6);
			}
			else
			{
				bsp_SetMotorSpeed(MotorLeft,6);
				bsp_SetMotorSpeed(MotorRight,5);
			}
		}break;
	}	
}






