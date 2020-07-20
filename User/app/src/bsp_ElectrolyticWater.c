#include "bsp.h"

#define RCC_ALL_ELECTROLYTIC_WATER 	(RCC_APB2Periph_GPIOC)

#define GPIO_PORT_ELECTROLYTIC_WATER  GPIOC
#define GPIO_PIN_ELECTROLYTIC_WATER	  GPIO_Pin_5


#define ELECTROLYTIC_WATER_ON()      VACUUM_1_AS_ELECTROLYTIC_H2O
#define ELECTROLYTIC_WATER_OFF()     VACUUM_0_AS_ELECTROLYTIC_H2O

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitLed
*	����˵��: ����LEDָʾ����ص�GPIO,  �ú����� bsp_Init() ���á�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitElectrolyticWater(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_ALL_ELECTROLYTIC_WATER, ENABLE);

	ELECTROLYTIC_WATER_OFF();
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* �������ģʽ */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_ELECTROLYTIC_WATER;
	GPIO_Init(GPIO_PORT_ELECTROLYTIC_WATER, &GPIO_InitStructure);
	
	ELECTROLYTIC_WATER_OFF();
}


typedef struct
{
	bool isRunning;
	uint32_t delay;
	uint32_t action;
	
	uint32_t cnt;
	uint32_t onUS;
}ElectrolyticWater;


static ElectrolyticWater electrolyticWater;


static inline void bsp_ELECTROLYTIC_WATER_ON(void);
static inline void bsp_ELECTROLYTIC_WATER_OFF(void);


static inline void bsp_ELECTROLYTIC_WATER_ON(void)
{
	
	if(!electrolyticWater.isRunning)
	{
		ELECTROLYTIC_WATER_OFF();
		return;
	}
	
	/*2MS  һ������   0-5S  35US��*/
	if(electrolyticWater.cnt <= (5000 / 2))
	{
		electrolyticWater.onUS = 35*2 ;
	}
	/*2MS  һ������   5-8S  760US��*/
	else if(electrolyticWater.cnt <= (8000 / 2) && electrolyticWater.onUS <= 760*2)
	{
		electrolyticWater.onUS +=2 ;
	}
	else if(electrolyticWater.cnt > (8000 / 2))
	{
		electrolyticWater.onUS = 760*2 ;
	}
	
	ELECTROLYTIC_WATER_ON();
	
	bsp_StartHardTimer(1, electrolyticWater.onUS, (void *)bsp_ELECTROLYTIC_WATER_OFF);
	
	
}

static inline void bsp_ELECTROLYTIC_WATER_OFF(void)
{
	if(!electrolyticWater.isRunning)
	{
		ELECTROLYTIC_WATER_OFF();
		return;
	}
	
	
	ELECTROLYTIC_WATER_OFF();
	
	bsp_StartHardTimer(1, 4000-electrolyticWater.onUS, (void *)bsp_ELECTROLYTIC_WATER_ON);
	
	/*2MS  һ������   ǰ��5S  35US��*/
	/*2MS  һ������   3S��  35US����Ϊ760US��*/
	++electrolyticWater.cnt;
}


void bsp_StartElectrolyticWaterProc(void)
{
	electrolyticWater.action = 0 ;
	electrolyticWater.delay = 0 ;
	electrolyticWater.cnt = 0 ;
	electrolyticWater.isRunning = true;
	
	bsp_ELECTROLYTIC_WATER_ON();
}


void bsp_StopElectrolyticWaterProc(void)
{
	electrolyticWater.isRunning = false;
	electrolyticWater.action = 0 ;
	electrolyticWater.delay = 0 ;
	electrolyticWater.cnt = 0 ;
	
	
	bsp_ELECTROLYTIC_WATER_OFF();
}




