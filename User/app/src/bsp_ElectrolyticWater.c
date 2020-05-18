#include "bsp.h"

#define RCC_ALL_ELECTROLYTIC_WATER 	(RCC_APB2Periph_GPIOB)

#define GPIO_PORT_ELECTROLYTIC_WATER  GPIOB
#define GPIO_PIN_ELECTROLYTIC_WATER	  GPIO_Pin_3


#define ELECTROLYTIC_WATER_ON()      GPIO_SetBits(GPIO_PORT_ELECTROLYTIC_WATER,GPIO_PIN_ELECTROLYTIC_WATER)
#define ELECTROLYTIC_WATER_OFF()     GPIO_ResetBits(GPIO_PORT_ELECTROLYTIC_WATER,GPIO_PIN_ELECTROLYTIC_WATER)

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

