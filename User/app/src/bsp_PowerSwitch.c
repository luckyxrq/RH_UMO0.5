#include "bsp.h"


/*
	���Ŷ�����ͷ�ļ���������͹�������ͬ��
*/


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
	bsp_SwOff(SW_VSLAM_POWER);
	bsp_SwOff(SW_WIFI_POWER);
	bsp_SwOff(SW_3V3_EN_CTRL);

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
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	/* ��©��� */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_VSLAM_POWER;
	GPIO_Init(GPIO_PORT_VSLAM_POWER, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* �������ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_WIFI_POWER;
	GPIO_Init(GPIO_PORT_WIFI_POWER, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* �������ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_3V3_EN_CTR;
	GPIO_Init(GPIO_PORT_3V3_EN_CTR, &GPIO_InitStructure);
	
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
		
		case SW_VSLAM_POWER:
		{
			GPIO_ResetBits(GPIO_PORT_VSLAM_POWER,GPIO_PIN_VSLAM_POWER);
		}break;
		
		case SW_WIFI_POWER:
		{
			GPIO_SetBits(GPIO_PORT_WIFI_POWER,GPIO_PIN_WIFI_POWER);
		}break;
		
		case SW_3V3_EN_CTRL:
		{
			GPIO_ResetBits(GPIO_PORT_3V3_EN_CTR,GPIO_PIN_3V3_EN_CTR);
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
		
		case SW_VSLAM_POWER:
		{
			GPIO_SetBits(GPIO_PORT_VSLAM_POWER,GPIO_PIN_VSLAM_POWER);
		}break;
		
		case SW_WIFI_POWER:
		{
			GPIO_ResetBits(GPIO_PORT_WIFI_POWER,GPIO_PIN_WIFI_POWER);
		}break;
		
		case SW_3V3_EN_CTRL:
		{
			GPIO_SetBits(GPIO_PORT_3V3_EN_CTR,GPIO_PIN_3V3_EN_CTR);
		}break;
		
		default: break;
	}

}

