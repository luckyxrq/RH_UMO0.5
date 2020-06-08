#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1

#define MAX_SEARCH_TICK         (1000*120)


/*ǰ���2������ͷ*/
#define FRONT_RX_L      IR_CH2
#define FRONT_RX_R      IR_CH1

#define IR_RX_L      IR_CH3
#define IR_RX_R      IR_CH4

/*ֱ��*/
#define CASE_RANDOM_0    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))
#define CASE_RANDOM_7    (!bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))
#define CASE_RANDOM_10    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))

/*���һ�����*/
#define CASE_RANDOM_11   ((bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_RIGHT)) && CASE_RANDOM_7)
/*���󻮴���*/
#define CASE_RANDOM_12   ((bsp_IR_GetRev(IR_RX_L,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_RX_L,IR_TX_SITE_RIGHT)) && CASE_RANDOM_7)


#define CASE_RANDOM_3    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))
#define CASE_RANDOM_4    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))

#define CASE_RANDOM_5    (!bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT))
#define CASE_RANDOM_6    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))

#define CASE_RANDOM_8    (bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))
#define CASE_RANDOM_9    (!bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT) && bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT))

#define CASE_RANDOM_13   ((bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_RX_L,IR_TX_SITE_RIGHT)) && CASE_RANDOM_7)
				

#define CASE_STOP_RANDOM (CASE_RANDOM_11 || CASE_RANDOM_12 || CASE_RANDOM_0 || CASE_RANDOM_10 || CASE_RANDOM_3 || CASE_RANDOM_4 || CASE_RANDOM_5 || CASE_RANDOM_6 || CASE_RANDOM_8 || CASE_RANDOM_9)

typedef enum
{
	eNone = 0 ,          /*û����ײ*/
	eHasSignalCollision, /*�ͳ��׮�����˺���ײ*/
	eNoSignalCollision   /*û���źŵ���ײ��û�����ϳ��׮������������ǽ��*/
}SearchChargePileCollision;

typedef struct
{
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
	
	uint32_t startTick;
	
	bool isNeedPlaySong;
	uint32_t isNeedPlaySongTick;
	uint32_t lastIsChargeDoneTick;
	uint32_t lastIsTouchTick;
	uint32_t lastIsNeedEdgeTick;
	
	bool isNeedFirstRunRandom;
	
	Collision collision;
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
	
	
	search.action = 0 ;
	search.delay = 0 ;
	search.startTick = xTaskGetTickCount();
	search.isNeedFirstRunRandom = true;
	search.isRunning = true;
	
	/*��ֹ����������*/

	bsp_IRD_StartWork();
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
	
	bsp_SetMotorSpeed(MotorLeft, 0);
	bsp_SetMotorSpeed(MotorRight,0);
	search.isRunning = false;
	search.isNeedFirstRunRandom = false;
	search.action = 0 ;
	search.delay = 0 ;
	
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
	
	bsp_IRD_StopWork();
}	



void bsp_DetectIsTouchChargePile(void)
{
	if(search.isRunning && bsp_IsTouchChargePile())
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		
		bsp_CloseAllStateRun();
		
		search.isRunning = false;
	}
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
	
	bsp_IsTouchChargePile();
	bsp_IsCharging();
	bsp_IsChargeDone();
	
	
	//DEBUG("%d %d  ---  %d %d\r\n",bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_LEFT),bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_RIGHT),bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_LEFT),bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_RIGHT));
	//DEBUG("%d %d  ---  %d %d\r\n",bsp_IR_GetRev(IR_RX_L,IR_TX_SITE_LEFT),bsp_IR_GetRev(IR_RX_L,IR_TX_SITE_RIGHT),bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_LEFT),bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_RIGHT));

	if(isCleanRunning())
		return;
	
	if(bsp_IsTouchChargePile())
	{
		/*���ſ�ʼ���*/
		if(search.isNeedPlaySong && (xTaskGetTickCount() >= search.isNeedPlaySongTick  && xTaskGetTickCount() - search.isNeedPlaySongTick >= 1000) ) /*���ʱ���жϱ����˶������ſ�ʼ���*/
		{
			search.isNeedPlaySongTick = UINT32_T_MAX; /*�������ʱ��̶ȣ��´���Ȼ��������*/
			bsp_SperkerPlay(Song22);
			search.isNeedPlaySong = false;
		}

		if(bsp_IsChargeDone())
		{
			
			if(xTaskGetTickCount() - search.lastIsChargeDoneTick >= 1000) /*���ʱ���жϱ����˻Ƶƣ��̵���˸*/
			{
				bsp_SetLedState(AT_CHARGE_DONE);
			}
		}
		else
		{
			bsp_SetLedState(AT_CHARGING);
			search.lastIsChargeDoneTick = xTaskGetTickCount();
		}
		
		search.lastIsTouchTick = xTaskGetTickCount();
	}
	else /*��׮״̬��Ҫ������ģ����ǵ���Ҫ�Ȼ��������Ȼ�ᶶ��*/
	{
		if((xTaskGetTickCount() >= search.lastIsTouchTick)  &&  (xTaskGetTickCount() - search.lastIsTouchTick >= 500))
		{
			 /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!������Ҫ����ɨ��������*/
			if( bsp_GetLedAppState() != AT_CLEAN && 
				bsp_GetLedAppState() != AT_SEARCH_CHARGE &&
				bsp_GetLedAppState() != THREE_WHITE_TOOGLE &&
			    bsp_GetLedAppState() != AT_LINK)
			{
				bsp_SetLedState(THREE_WHITE_ON);
			}
			
			search.isNeedPlaySongTick = xTaskGetTickCount();
			search.isNeedPlaySong = true;
			search.lastIsTouchTick = UINT32_T_MAX;
		}
		
	}
	
	
	if(!search.isRunning)
	{
		return;
	}
		

	/*��ʱ��ʱ�䵽�˻�û�ҵ����׮*/
	if(xTaskGetTickCount() - search.startTick >= MAX_SEARCH_TICK)
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_CloseAllStateRun();
		
		bsp_SetLedState(THREE_WHITE_ON); 
		bsp_SperkerPlay(Song24);
		
		return;
	}
	
	
	
	/*���*/
	if(bsp_IsTouchChargePile() == true)
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_CloseAllStateRun();
		
		return ;
	}
	
	/*�������ɨ*/
	if(search.isNeedFirstRunRandom)
	{
		bsp_StartStrategyRandom();
		search.isNeedFirstRunRandom = false;
	}
	
	
	
	/*�����ִ�������ɨ����ִ�к����Ѱ�ҳ��׮*/
	if(bsp_IsStartStrategyRandom())
	{
		if(CASE_STOP_RANDOM) /*�յ����źž�ִ�к����Ѱ�ҳ��׮*/
		{
			DEBUG("�˳����\r\n");
			
			bsp_SetMotorSpeed(MotorLeft,0);
			bsp_SetMotorSpeed(MotorRight,0);
			bsp_StopStrategyRandom();
		}
		return;
	}
	
	
	switch(search.action)
	{
		case 0:
		{
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,6);
			
			search.action++;
		}break;
		
		case 1:
		{
			/*�����ж���ײ*/
			search.collision = bsp_CollisionScan();
			
			//if(ret != CollisionNone || bsp_CliffIsDangerous(CliffLeft) || bsp_CliffIsDangerous(CliffMiddle) || bsp_CliffIsDangerous(CliffRight))
			if(search.collision != CollisionNone )
			{
				/*������������˾ͺ��ˣ��ں��˵Ĺ�����������������*/
				bsp_SetMotorSpeed(MotorLeft, -3);
				bsp_SetMotorSpeed(MotorRight,-3);
				
				search.delay = xTaskGetTickCount();
				search.action++;
			}
			else
			{
				#define CASE_RANDOM_15   (bsp_IR_GetRev(IR_RX_R,IR_TX_SITE_CENTER) && !bsp_IR_GetRev(FRONT_RX_L,IR_TX_SITE_CENTER) && !bsp_IR_GetRev(FRONT_RX_R,IR_TX_SITE_CENTER))
				
				if(CASE_RANDOM_11) /*���� �����յ����׮��2��խ���е��κ�һ�� �� ����ǰ��û���յ��ź�*/
				{
					bsp_SetMotorSpeed(MotorLeft, 8);
					bsp_SetMotorSpeed(MotorRight,0);
				}
				else if(CASE_RANDOM_12) /*���� �����յ����׮��2��խ���е��κ�һ�� �� ����ǰ��û���յ��ź�*/
				{
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,8);
				}
				else if(CASE_RANDOM_0 ||  CASE_RANDOM_10) /*������ͬʱ���� ���� ����ͬʱ���ң� ����  ��������������  �������Ҳ�������*/
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,3);
				}
				else if(CASE_RANDOM_3 || CASE_RANDOM_6 || CASE_RANDOM_8)
				{
					/*���������ң��������Ҳ�������*/
					/*�������ң������Ҳ�������Ҳ��������*/
					/*����ͬʱ���� �ҼȲ�����Ҳ������*/
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,4);
				}
				else if(CASE_RANDOM_4 || CASE_RANDOM_5 || CASE_RANDOM_9)
				{
					/*������  ������ ���� ������*/
					/*��Ȳ�����Ҳ������  ������*/
					/*��Ȳ�����Ҳ�����ң�����ͬʱ����*/
					bsp_SetMotorSpeed(MotorLeft, 5);
					bsp_SetMotorSpeed(MotorRight,3);
				}
				
				else if(CASE_RANDOM_15)
				{
					/*�ұ߽������յ����  ǰ�������ղ������*/
					bsp_SetMotorSpeed(MotorLeft, 5);
					bsp_SetMotorSpeed(MotorRight,3);
				}
				
				else if(CASE_RANDOM_7) /*���ղ��������κ�һ�� ���� ���ղ��������κ�һ��*/
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,3);
				}
			}
		}break;
		

		case 2:
		{
			if(xTaskGetTickCount() - search.delay >= 2000)
			{
				search.action = 0 ;
			}
			else
			{
				if(CASE_RANDOM_3 || CASE_RANDOM_6 || CASE_RANDOM_8)
				{
					bsp_SetMotorSpeed(MotorLeft, -5);
					bsp_SetMotorSpeed(MotorRight,-2);
				}
				else if(CASE_RANDOM_4 || CASE_RANDOM_5 || CASE_RANDOM_9)
				{
					bsp_SetMotorSpeed(MotorLeft, -2);
					bsp_SetMotorSpeed(MotorRight,-5);
				}
			}
		}break;

	}
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitIO
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

