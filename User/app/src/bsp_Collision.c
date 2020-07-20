#include "bsp.h"


/* ����ײ�������ض�Ӧ��RCCʱ�� */
#define RCC_ALL_ANTICOLLISION 	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_LEFT   GPIOA
#define GPIO_PIN_LEFT	 GPIO_Pin_5

#define GPIO_PORT_RIGHT  GPIOF
#define GPIO_PIN_RIGHT	 GPIO_Pin_8




/*
*********************************************************************************************************
*	�� �� ��: bsp_InitCollision
*	����˵��: ��ʼ����ײ�������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitCollision(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_ALL_ANTICOLLISION, ENABLE);


	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; /* �������룬������ʱ���ⲿ���ͣ���ײʱ�ָ��ߵ�ƽ */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LEFT;
	GPIO_Init(GPIO_PORT_LEFT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RIGHT;
	GPIO_Init(GPIO_PORT_RIGHT, &GPIO_InitStructure);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_CollisionScan
*	����˵��: ������ײ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
Collision bsp_CollisionScan(void)
{
	uint8_t ret = 0 ;
	
	/* ���ײ���� */
	if(GPIO_ReadInputDataBit(GPIO_PORT_LEFT,GPIO_PIN_LEFT))
	{
		ret |= 1<<0;
	}
	
	
	/* �ұ�ײ���� */
	if(GPIO_ReadInputDataBit(GPIO_PORT_RIGHT,GPIO_PIN_RIGHT))
	{
		ret |= 1<<1;
	}
	
	if(ret == 0x00)
	{
		return CollisionNone;
	}
	else if(ret == 0x01)
	{
		return CollisionLeft;
	}
	else if(ret == 0x02)
	{
		return CollisionRight;
	}
	else if(ret == 0x03)
	{
		return CollisionAll;
	}
	else
	{
		WARNING("���������ײ���\r\n");
		return CollisionNone;
	}
	
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_PrintCollision
*	����˵��: ��ӡ��ײ��Ϣ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_PrintCollision(void)
{
	if(bsp_CollisionScan() == CollisionLeft)
	{
		DEBUG("���\r\n");
	}
	else if(bsp_CollisionScan() == CollisionRight)
	{
		DEBUG("�ұ�\r\n");
	}
	else if(bsp_CollisionScan() == CollisionAll)
	{
		DEBUG("����\r\n");
	}
}


