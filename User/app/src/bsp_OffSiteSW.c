#include "bsp.h"

#define RCC_ALL_OFFSITE_SW 	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE)

#define GPIO_PORT_OFFSITE_SW_L  GPIOA
#define GPIO_PIN_OFFSITE_SW_L   GPIO_Pin_12


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


void bsp_PrintOffSiteState(void)
{
	OffSiteState ret = bsp_OffSiteGetState();
	
	DEBUG("��أ�");
	if(ret == OffSiteBoth)
	{
		DEBUG("����");
	}
	else if(ret == OffSiteLeft)
	{
		DEBUG("��");
	}
	else if(ret == OffSiteRight)
	{
		DEBUG("��");
	}
	else
	{
		DEBUG("��");
	}
	DEBUG(" ");

}


typedef struct
{
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
}OffSiteProc;

static OffSiteProc offSiteProc;


void bsp_StartOffSiteProc(void)
{
	offSiteProc.action = 0 ;
	offSiteProc.delay = 0 ;
	offSiteProc.isRunning = true;
}

void bsp_StopOffSiteProc(void)
{
	offSiteProc.isRunning = false;
	offSiteProc.action = 0 ;
	offSiteProc.delay = 0 ;
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_OffSiteProc
*	����˵��: ��ؿ��ش�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_OffSiteProc(void)
{
	OffSiteState state;
	
	if(!offSiteProc.isRunning)
		return;
	
	switch(offSiteProc.action)
	{
		case 0:
		{
			state = bsp_OffSiteGetState();
			if(state != OffSiteNone)
			{
				/*��������*/
				bsp_SperkerPlay(Song16);

				DEBUG("��ؿ���\r\n");
	
				/*�ƹ�ָ��ʼ*/
				bsp_LedOn(LED_LOGO_CLEAN);
				bsp_LedOn(LED_LOGO_POWER);
				bsp_LedOn(LED_LOGO_CHARGE);
				bsp_LedOff(LED_COLOR_YELLOW);
				bsp_LedOff(LED_COLOR_GREEN);
				bsp_LedOff(LED_COLOR_RED);
				
				bsp_StopRunToggleLED();
				
				/*��λ��һ�εİ���״̬*/
				bsp_SetKeyRunLastState(RUN_STATE_DEFAULT);
				
				/*�رո���״̬��*/
				bsp_StopCliffTest();
				bsp_StopVacuum();
				/*�رյ��*/
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				
				offSiteProc.action++;
			}
		}break;
		
		case 1: /*�ȴ�����������*/
		{
			state = bsp_OffSiteGetState();
			if(state == OffSiteNone)
			{
				offSiteProc.action = 0 ;
			}
		}break;
	}
}

