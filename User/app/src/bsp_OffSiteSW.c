#include "bsp.h"

#define RCC_ALL_OFFSITE_SW 	(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE)

#define GPIO_PORT_OFFSITE_SW_L  GPIOC
#define GPIO_PIN_OFFSITE_SW_L   GPIO_Pin_9


#define GPIO_PORT_OFFSITE_SW_R  GPIOE
#define GPIO_PIN_OFFSITE_SW_R   GPIO_Pin_12


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitOffSiteSW
*	����˵��: ��ʼ����ؿ���ʹ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitOffSiteSW(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_ALL_OFFSITE_SW, ENABLE);

	/*
		��ʼ״̬�ȹر�
	*/
	bsp_SwOff(SW_5V_EN_CTRL);
	bsp_SwOff(SW_IR_POWER);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	/* �������� */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_OFFSITE_SW_L;
	GPIO_Init(GPIO_PORT_OFFSITE_SW_L, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	/* �������� */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_OFFSITE_SW_R;
	GPIO_Init(GPIO_PORT_OFFSITE_SW_R, &GPIO_InitStructure);

}


/*
*********************************************************************************************************
*	�� �� ��: bsp_OffSiteGetState
*	����˵��: ���ص�ǰ��ؿ���״̬
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
OffSiteState bsp_OffSiteGetState(void)
{
	OffSiteState ret ;
	
	if(!GPIO_ReadInputDataBit(GPIO_PORT_OFFSITE_SW_L,GPIO_PIN_OFFSITE_SW_L)&&
		!GPIO_ReadInputDataBit(GPIO_PORT_OFFSITE_SW_R,GPIO_PIN_OFFSITE_SW_R))
	{
		ret = OffSiteBoth;
	}
	else if(!GPIO_ReadInputDataBit(GPIO_PORT_OFFSITE_SW_L,GPIO_PIN_OFFSITE_SW_L))
	{
		ret = OffSiteLeft;
	}
	else if(!GPIO_ReadInputDataBit(GPIO_PORT_OFFSITE_SW_R,GPIO_PIN_OFFSITE_SW_R))
	{
		ret = OffSiteRight;
	}
	else
	{
		ret = OffSiteNone;
	}
	
	return ret;
}

