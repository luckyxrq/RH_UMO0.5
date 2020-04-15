#include "bsp.h"

/* ����ײ�������ض�Ӧ��RCCʱ�� */
#define RCC_ALL_DUST_BOX 	(RCC_APB2Periph_GPIOG)

#define GPIO_PORT_DUST_BOX   GPIOG
#define GPIO_PIN_DUST_BOX	 GPIO_Pin_14





/*
*********************************************************************************************************
*	�� �� ��: bsp_InitDustBox
*	����˵��: ��ʼ�����м��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitDustBox(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_ALL_DUST_BOX, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; /* �������� */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_DUST_BOX;
	GPIO_Init(GPIO_PORT_DUST_BOX, &GPIO_InitStructure);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_DustBoxGetState
*	����˵��: ���س���״̬
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
DustBoxState bsp_DustBoxGetState(void)
{
	DustBoxState ret ;
	
	if(GPIO_ReadInputDataBit(GPIO_PORT_DUST_BOX,GPIO_PIN_DUST_BOX))
	{
		ret = DustBoxInside;
	}
	else
	{
		ret = DustBoxOutside;
	}
	
	return ret ;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_DustBoxGetState
*	����˵��: ���س���״̬
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_DustBoxProc(void)
{
	static DustBoxState lastState = DustBoxInside;
	
	DustBoxState state = bsp_DustBoxGetState(); /*��ȡ����״̬*/
	
	if(lastState != state)
	{
		lastState = state;
		if(state == DustBoxInside)
		{
			bsp_SperkerPlay(Song10);
		}
		else if(state == DustBoxOutside)
		{
			bsp_SperkerPlay(Song9);
		}
	}
	else
	{
		
	}
}


