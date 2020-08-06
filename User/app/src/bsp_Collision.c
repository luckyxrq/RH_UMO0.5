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
		RTT("L\r\n");
	}
	else if(bsp_CollisionScan() == CollisionRight)
	{
		RTT("R\r\n");
	}
	else if(bsp_CollisionScan() == CollisionAll)
	{
		RTT("BOTH\r\n");
	}
}


