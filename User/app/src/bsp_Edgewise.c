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
//	Collision collision;
//	

//	switch(edgewise.action)
//	{
//		case 0:/*�ж��Ƿ�����ر�ģʽ*/
//		{

//		}

//	}	
}






