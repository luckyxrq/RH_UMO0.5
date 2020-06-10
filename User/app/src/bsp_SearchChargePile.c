#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1

#define MAX_SEARCH_TICK         (1000*120)

#define REAL_ANGLE()      (bsp_AngleReadRaw()*0.01F)

#define _SEARCH_PILE_GO_BACK_PULSE                  (10/(3.14F*70)*1024)

/*
**********************************************************************************************************
                                            ֱ��
**********************************************************************************************************
*/
#define RUN_STRAIGHT_0      ( IR_FL_L && IR_FL_R && IR_FR_L && IR_FR_R    )  /*ǰ�����������յ�2��խ���ź�*/
#define RUN_STRAIGHT_1      ( IR_FL_L && !IR_FL_R && !IR_FR_L && IR_FR_R  )  /*ǰ�����������յ���֮���Ե�խ���ź�*/


/*
**********************************************************************************************************
                                     ǰ��û���յ��κ�խ���ź�
**********************************************************************************************************
*/
#define F_NO_NARROW_SIGNAL  ( !IR_FL_L && !IR_FL_R && !IR_FR_L && !IR_FR_R  ) /*ǰ���������ղ����κ�խ���ź�*/


/*
**********************************************************************************************************
                                    �����յ��˹��  ��ҪСһ��Ĵ�ת��
**********************************************************************************************************
*/
#define ROTATE_CW_LITTLE    ( !(IR_SR_L && IR_SR_R) && IR_SR_M && F_NO_NARROW_SIGNAL)
#define ROTATE_CCW_LITTLE   ( !(IR_SL_L && IR_SL_R) && IR_SL_M && F_NO_NARROW_SIGNAL)

/*
**********************************************************************************************************
                                    �����յ���2��խ��  ��Ҫ��ת��
**********************************************************************************************************
*/
#define ROTATE_CW           ( (IR_SR_L && IR_SR_R) && F_NO_NARROW_SIGNAL)
#define ROTATE_CCW          ( (IR_SL_L && IR_SL_R) && F_NO_NARROW_SIGNAL)

/*
**********************************************************************************************************
                                     �Ѿ�����������ǰ�棬׼��΢��
**********************************************************************************************************
*/
#define INCLINATION_GO_L_0    ( IR_FL_L && IR_FL_R && !IR_FR_L && IR_FR_R    )  /*���������ң��������Ҳ�������*/
#define INCLINATION_GO_L_1    ( IR_FL_R && !IR_FR_L && !IR_FR_R    )            /*�������ң������Ҳ�������Ҳ��������*/
#define INCLINATION_GO_L_2    ( IR_FL_L && IR_FL_R && !IR_FR_L && !IR_FR_R    ) /*����ͬʱ���� �ҼȲ�����Ҳ������*/

#define INCLINATION_GO_R_0    ( IR_FL_L && !IR_FL_R && IR_FR_L && IR_FR_R    )  /*������  ������ ���� ������Ҳ����*/
#define INCLINATION_GO_R_1    ( !IR_FL_L && !IR_FL_R && IR_FR_L              )  /*��Ȳ�����Ҳ������  ������*/
#define INCLINATION_GO_R_2    ( !IR_FL_L && !IR_FL_R && IR_FR_L && IR_FR_R   )  /*��Ȳ�����Ҳ�����ң�����ͬʱ����*/

/*
**********************************************************************************************************
                  ͷ���յ��˹�ǵ���û���յ�խ�ǣ�����û���κ��ź�(����˹ԭ����ת�������һ�����)
**********************************************************************************************************
*/
#define SL_NO_SIGNAL          (!IR_SL_L && !IR_SL_R && !IR_SL_M)  /*���ϵ���� û���κ��ź�*/
#define SR_NO_SIGNAL          (!IR_SR_L && !IR_SR_R && !IR_SR_M)  /*���ϵ��ұ� û���κ��ź�*/

#define FL_NO_SIGNAL          (!IR_FL_L && !IR_FL_R && !IR_FL_M)  /*ǰ�����û���κ��ź�*/
#define FR_NO_SIGNAL          (!IR_FR_L && !IR_FR_R && !IR_FR_M)  /*ǰ���ұ�û���κ��ź�*/


#define ONLY_F_RX_WIDE        ( (IR_FL_M ||  IR_FR_M) && F_NO_NARROW_SIGNAL && SL_NO_SIGNAL && SR_NO_SIGNAL)    

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
	
	uint32_t pulse;
	float angle;
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
		if(1) /*�յ����źž�ִ�к����Ѱ�ҳ��׮*/
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
				if(ONLY_F_RX_WIDE)
				{
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					
					search.action = 3 ;
				}
				if(ROTATE_CW_LITTLE)
				{
					bsp_SetMotorSpeed(MotorLeft, 6);
					bsp_SetMotorSpeed(MotorRight,3);
				}
				else if(ROTATE_CW_LITTLE)
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,6);
				}
				else if(ROTATE_CW)
				{
					bsp_SetMotorSpeed(MotorLeft, 4);
					bsp_SetMotorSpeed(MotorRight,-4);
				}
				else if(ROTATE_CCW)
				{
					bsp_SetMotorSpeed(MotorLeft, -4);
					bsp_SetMotorSpeed(MotorRight,4);
				}
				else if(RUN_STRAIGHT_0 ||  RUN_STRAIGHT_1)
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,3);
				}
				else if(INCLINATION_GO_L_0 || INCLINATION_GO_L_1 || INCLINATION_GO_L_2)
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,5);
				}
				else if(INCLINATION_GO_R_0 || INCLINATION_GO_R_1 || INCLINATION_GO_R_2)
				{
					bsp_SetMotorSpeed(MotorLeft, 5);
					bsp_SetMotorSpeed(MotorRight,3);
				}
				else if(F_NO_NARROW_SIGNAL) 
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,3);
				}
			}
		}break;
		

		case 2:
		{
			if(xTaskGetTickCount() - search.delay >= 3900)
			{
				search.action = 0 ;
			}
			else
			{
				if(INCLINATION_GO_L_0 || INCLINATION_GO_L_1 || INCLINATION_GO_L_2)
				{
					bsp_SetMotorSpeed(MotorLeft, -5);
					bsp_SetMotorSpeed(MotorRight,-3);
				}
				else if(INCLINATION_GO_R_0 || INCLINATION_GO_R_1 || INCLINATION_GO_R_2)
				{
					bsp_SetMotorSpeed(MotorLeft, -3);
					bsp_SetMotorSpeed(MotorRight,-5);
				}
				else if(F_NO_NARROW_SIGNAL) 
				{
					bsp_SetMotorSpeed(MotorLeft, -3);
					bsp_SetMotorSpeed(MotorRight,-3);
				}
			}
		}break;
		
		case 3:
		{
			search.angle = REAL_ANGLE();
			bsp_SetMotorSpeed(MotorLeft, -3);
			bsp_SetMotorSpeed(MotorRight,3);
			
			++search.action;
		}break;
		
		case 4:
		{
			if(ABS(REAL_ANGLE() - bsp_AngleAdd(search.angle, 90)) <= 10)
			{
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,2);
				search.delay = xTaskGetTickCount();
				++search.action;
			}
		}break;
		
		
		case 5:
		{
			if(xTaskGetTickCount() - search.delay >= 3600  || bsp_CollisionScan() != CollisionNone)
			{
				
				if(SL_NO_SIGNAL && SR_NO_SIGNAL && FL_NO_SIGNAL && FR_NO_SIGNAL)  /*���Ǵ�ģ��򻻱�*/
				{
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					
					++search.action;
				}
				else
				{
					search.action = 0;
				}
			}
			else if(INCLINATION_GO_L_0 || INCLINATION_GO_L_1 || INCLINATION_GO_L_2 || INCLINATION_GO_R_0 || INCLINATION_GO_R_1 || INCLINATION_GO_R_2 || ROTATE_CW || ROTATE_CCW)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				search.action = 0;
			}
		}break;
		
		case 6:
		{
			search.pulse = bsp_GetCurrentBothPulse();
			bsp_SetMotorSpeed(MotorLeft, -3);
			bsp_SetMotorSpeed(MotorRight,-3);
			++search.action;
		}break;
		
		case 7:
		{
			if(bsp_GetCurrentBothPulse() - search.pulse >= _SEARCH_PILE_GO_BACK_PULSE)
			{
				search.angle = REAL_ANGLE();
				bsp_SetMotorSpeed(MotorLeft, 3);
				bsp_SetMotorSpeed(MotorRight,-3);
				++search.action;
			}
		}break;
		
		case 8:
		{
			if(ABS(REAL_ANGLE() - bsp_AngleAdd(search.angle, 180)) <= 10)
			{
				bsp_SetMotorSpeed(MotorLeft, 2);
				bsp_SetMotorSpeed(MotorRight,6);
				search.delay = xTaskGetTickCount();
				++search.action;
			}
		}break;
		
		case 9:
		{
			if(xTaskGetTickCount() - search.delay >= 3600 )
			{
				search.action = 1 ;
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

