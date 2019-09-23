#include "bsp.h"


#define RCC_ALL_SW 	(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_5V_EN_CTR  GPIOE
#define GPIO_PIN_5V_EN_CTR   GPIO_Pin_15


#define GPIO_PORT_IR_POWER  GPIOF
#define GPIO_PIN_IR_POWER   GPIO_Pin_0


#define GPIO_PORT_MOTOR_POWER  GPIOA
#define GPIO_PIN_MOTOR_POWER   GPIO_Pin_9


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitSW
*	����˵��: ��ʼ����ѹʹ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitSW(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_ALL_SW, ENABLE);

	/*
		��ʼ״̬�ȹر�
	*/
	bsp_SwOff(SW_5V_EN_CTRL);
	bsp_SwOff(SW_IR_POWER);
	bsp_SwOff(SW_MOTOR_POWER);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* �������ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_5V_EN_CTR;
	GPIO_Init(GPIO_PORT_5V_EN_CTR, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	/* ��©���ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_IR_POWER;
	GPIO_Init(GPIO_PORT_IR_POWER, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* �������ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_MOTOR_POWER;
	GPIO_Init(GPIO_PORT_MOTOR_POWER, &GPIO_InitStructure);
	
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_SwOn
*	����˵��: ������ѹʹ��
*	��    ��:  sw : ʹ�ܶ�ID
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SwOn(SW_ID sw)
{
	switch(sw)
	{
		case SW_5V_EN_CTRL:
		{
			GPIO_SetBits(GPIO_PORT_5V_EN_CTR,GPIO_PIN_5V_EN_CTR);
		}break;
		
		case SW_IR_POWER:
		{
			GPIO_ResetBits(GPIO_PORT_IR_POWER,GPIO_PIN_IR_POWER);
		}break;
		
		case SW_MOTOR_POWER:
		{
			GPIO_SetBits(GPIO_PORT_MOTOR_POWER,GPIO_PIN_MOTOR_POWER);
		}break;
		
		default: break;
	}

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SwOff
*	����˵��: �رյ�ѹʹ��
*	��    ��:  sw : ʹ�ܶ�ID
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SwOff(SW_ID sw)
{
	switch(sw)
	{
		case SW_5V_EN_CTRL:
		{
			GPIO_ResetBits(GPIO_PORT_5V_EN_CTR,GPIO_PIN_5V_EN_CTR);
		}break;
		
		case SW_IR_POWER:
		{
			GPIO_SetBits(GPIO_PORT_IR_POWER,GPIO_PIN_IR_POWER);
		}break;
		
		case SW_MOTOR_POWER:
		{
			GPIO_ResetBits(GPIO_PORT_MOTOR_POWER,GPIO_PIN_MOTOR_POWER);
		}break;
		
		default: break;
	}

}

