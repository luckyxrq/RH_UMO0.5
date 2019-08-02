#include "bsp.h"

/* �ڶ�Ӧ��RCCʱ�� */
#define RCC_ALL_PULSE 	(RCC_APB2Periph_GPIOB)

#define GPIO_PORT_PULSE  GPIOB
#define GPIO_PIN_PULSE	 GPIO_Pin_0


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitPinPulse
*	����˵��: ��ʼ���������ţ�����ʾ����Э�����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitPinPulse(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_ALL_PULSE, ENABLE );	 
                      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_PULSE;
	GPIO_Init(GPIO_PORT_PULSE, &GPIO_InitStructure);	
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_SendPulse
*	����˵��: ����һ��10US�ĸߵ�ƽ����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SendPulse(void)
{
	GPIO_SetBits(GPIO_PORT_PULSE,GPIO_PIN_PULSE);
	bsp_DelayUS(10);
	GPIO_ResetBits(GPIO_PORT_PULSE,GPIO_PIN_PULSE);
}

