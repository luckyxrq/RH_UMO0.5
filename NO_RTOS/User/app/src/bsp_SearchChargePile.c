#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1



typedef struct
{
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
	
}Serach;


static Serach search;


/*
*********************************************************************************************************
*	�� �� ��: bsp_StartSearchChargePile
*	����˵��: ����Ѱ�ҳ��׮״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartSearchChargePile(void)
{
	bsp_IRD_StartWork();
	
	search.action = 0 ;
	search.delay = 0 ;
	search.isRunning = true;
	
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StopSearchChargePile
*	����˵��: ֹͣѰ�ҳ��׮״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StopSearchChargePile(void)
{
	search.isRunning = false;
	search.action = 0 ;
	search.delay = 0 ;
	
	bsp_IRD_StopWork();
}	


/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchChargePile
*	����˵��: Ѱ�ҳ��׮״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SearchChargePile(void)
{
	if(bsp_IsTouchChargePile())
	{
		/*���ſ�ʼ���*/
		//bsp_SperkerPlay(Song22);
		if(bsp_IsCharging())
		{
			bsp_SetLedState(LED_DEFAULT_STATE);
			bsp_CloseAllLed();
			bsp_LedOn(LED_COLOR_YELLOW);
			
			DEBUG("is charging\r\n");
		}
		else
		{
			bsp_SetLedState(LED_DEFAULT_STATE);
			bsp_CloseAllLed();
			bsp_LedOn(LED_COLOR_GREEN);
			
			DEBUG("charge done\r\n");
		}
	}
	else
	{
		bsp_LedOff(LED_COLOR_YELLOW);
		bsp_LedOff(LED_COLOR_GREEN);
	}
}

	
/*
*********************************************************************************************************
*	�� �� ��: bsp_IsTouchChargePile
*	����˵��: ��ȡ��練��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
bool bsp_IsTouchChargePile(void)
{
	if(GPIO_ReadInputDataBit(GPIO_PORT_CHARGE_TOUCH_PILE,GPIO_PIN_CHARGE_TOUCH_PILE))
	{
		return true ;
	}
	else
	{
		return false ;
	}
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_IsCharging
*	����˵��: �Ƿ��Ѿ�����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
bool bsp_IsCharging(void)
{
	if(GPIO_ReadInputDataBit(GPIO_PORT_CHARGE_IS_CHARGING,GPIO_PIN_CHARGE_IS_CHARGING)==0)
	{
		return true ;
	}
	else
	{
		return false ;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_IsChargeDone
*	����˵��: �Ƿ��Ѿ�����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
bool bsp_IsChargeDone(void)
{
	if(GPIO_ReadInputDataBit(GPIO_PORT_CHARGE_IS_DONE,GPIO_PIN_CHARGE_IS_DONE)==0)
	{
		return true ;
	}
	else
	{
		return false ;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitChargeIO
*	����˵��: ��ʼ�������ص�IO
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitChargeIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	
	/*��׮���*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CHARGE_TOUCH_PILE;
	GPIO_Init(GPIO_PORT_CHARGE_TOUCH_PILE, &GPIO_InitStructure);
	
	/*���ڳ����*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CHARGE_IS_CHARGING;
	GPIO_Init(GPIO_PORT_CHARGE_IS_CHARGING, &GPIO_InitStructure);
	
	/*������*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CHARGE_IS_DONE;
	GPIO_Init(GPIO_PORT_CHARGE_IS_DONE, &GPIO_InitStructure);
	
}

