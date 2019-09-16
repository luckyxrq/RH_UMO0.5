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
#if 0 /*û�к���Э������ײ*/
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
	
#else /*�к���Э������ײ*/
	
	Collision ret;
	
	if(GPIO_ReadInputDataBit(GPIO_PORT_LEFT,GPIO_PIN_LEFT) && GPIO_ReadInputDataBit(GPIO_PORT_RIGHT,GPIO_PIN_RIGHT))
	{
		ret = CollisionAll;
	}
	else if(GPIO_ReadInputDataBit(GPIO_PORT_LEFT,GPIO_PIN_LEFT) && bsp_AssistIsFaceObstacles())
	{
		ret = CollisionAll;
	}
	else if(GPIO_ReadInputDataBit(GPIO_PORT_RIGHT,GPIO_PIN_RIGHT) && bsp_AssistIsFaceObstacles())
	{
		ret = CollisionAll;
	}
	else if(GPIO_ReadInputDataBit(GPIO_PORT_LEFT,GPIO_PIN_LEFT))
	{
		ret = CollisionLeft;
	}
	else if(GPIO_ReadInputDataBit(GPIO_PORT_RIGHT,GPIO_PIN_RIGHT))
	{
		ret = CollisionRight;
	}
	else
	{
		ret = CollisionNone;
	}
	
	return ret;
	
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_CollisionDemo
*	����˵��: ��ײ����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_CollisionDemo(void)
{
	Collision ret = bsp_CollisionScan();
	
	switch(ret)
	{
		case CollisionLeft:
		{
			DEBUG("��߱���ײ��\r\n");
		}break;
		
		case CollisionRight:
		{
			DEBUG("�ұ߱���ײ��\r\n");
		}break;
		
		case CollisionAll:
		{
			DEBUG("���߱���ײ��\r\n");
		}break;
		
		case CollisionNone:
		{
		
		}break;
	}
}



