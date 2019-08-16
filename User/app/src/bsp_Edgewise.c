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

extern uint8_t flgdec ;

void bsp_EdgewiseAct(void)
{
//	Collision collision;
//	

//	switch(edgewise.action)
//	{
//		case 0:/*判断是否进入沿边模式*/
//		{

//		}

//	}	
}






