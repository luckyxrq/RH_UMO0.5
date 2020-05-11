#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1


#define STRAIGHT_SPEED_FAST      12
#define STRAIGHT_SPEED_SLOW      3

#define TURN_RIGHT_SPEED_FAST_L  5
#define TURN_RIGHT_SPEED_FAST_R  4

//
#define TURN_RIGHT_SPEED_SLOW_L  6
#define TURN_RIGHT_SPEED_SLOW_R  1


#define TURN_LEFT_SPEED_FAST_L   3
#define TURN_LEFT_SPEED_FAST_R   5
       
//	   
#define TURN_LEFT_SPEED_SLOW_L   1
#define TURN_LEFT_SPEED_SLOW_R   6

#define PIROUETTE_SPEED          4

#define BACKWARD_SPEED           -4

#define ROTATE_CCW_SPEED_L             -5
#define ROTATE_CCW_SPEED_R             5

#define MAX_SEARCH_TICK         (1000*120)

#define CORRECTION_ANGLE_TIME    200

                              /*ǰ��2���������յ����ҷ���*/
#define CASE_STOP_RANDOM_0    (bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)&& bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
                              /*ǰ��2�������ո�*/
#define CASE_STOP_RANDOM_1    (bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
                              /*1�����յ�2�� ,2�����յ����*/
#define CASE_STOP_RANDOM_2    (bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
                              /*2�����յ�2�� ,1�����յ��ұ�*/
#define CASE_STOP_RANDOM_3    (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
                              /*1��2�Ŷ����յ���ߣ��������յ��ұ�*/
#define CASE_STOP_RANDOM_4    (bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
                              /*1��2�Ŷ����յ��ұߣ��������յ����*/
#define CASE_STOP_RANDOM_5    (bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
                              /*1�Ų���ͬʱ�յ����ҷ��䣬2��ͬʱ�յ����ҷ���*/
#define CASE_STOP_RANDOM_6    (!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))&& (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
                              /*1����ͬʱ�յ����ҷ��䣬2����ͬʱ�յ����ҷ���*/
#define CASE_STOP_RANDOM_7    ((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
                              /*����4�����յ���Ǻ�����ң�����1,2�������յ��κ�,ԭ����ת*/
#define CASE_STOP_RANDOM_8    (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)))
                              /*����3�����յ���Ǻ�����ң�����1,2�������յ��κ�,ԭ����ת*/
#define CASE_STOP_RANDOM_9    (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT)))



#define CASE_STOP_RANDOM      (CASE_STOP_RANDOM_0 || CASE_STOP_RANDOM_1 || CASE_STOP_RANDOM_2 || CASE_STOP_RANDOM_3 || CASE_STOP_RANDOM_4 || CASE_STOP_RANDOM_5 || CASE_STOP_RANDOM_6 || CASE_STOP_RANDOM_7 || CASE_STOP_RANDOM_8 || CASE_STOP_RANDOM_9)

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
	uint32_t lastIsChargingTick;
	uint32_t lastIsTouchTick;
	uint32_t lastIsNeedEdgeTick;
	
	bool isNeedFirstRunRandom;
}Serach;


static Serach search;
static void bsp_InitIO(void);
static void bsp_SearchRunStraightFast(void);
static void bsp_SearchRunStraightSlow(void);
static void bsp_SearchTurnRightFast(void)  ;
static void bsp_SearchTurnRightSlow(void)  ;
static void bsp_SearchTurnLeftFast(void)   ;
static void bsp_SearchTurnLeftSlow(void)   ;
static void bsp_PirouetteCW(void)          ;
static void bsp_PirouetteCCW(void)         ;
static void bsp_GoBackward(void)           ;

static void bsp_SearchRunStraightFastBack(void);
static void bsp_SearchRunStraightSlowBack(void);
static void bsp_SearchTurnRightFastBack(void);
static void bsp_SearchTurnRightSlowBack(void);
static void bsp_SearchTurnLeftFastBack(void);
static void bsp_SearchTurnLeftSlowBack(void);


static void bsp_RotateCCW(void);
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
	bsp_InitIO();
	
	search.action = 0 ;
	search.delay = 0 ;
	search.startTick = xTaskGetTickCount();
	search.isNeedFirstRunRandom = true;
	search.isRunning = true;
	
	/*��ֹ����������*/
	UNUSED(bsp_SearchRunStraightFast);
	UNUSED(bsp_SearchRunStraightSlow);
	UNUSED(bsp_SearchTurnRightFast)  ;
	UNUSED(bsp_SearchTurnRightSlow)  ;
	UNUSED(bsp_SearchTurnLeftFast)   ;
	UNUSED(bsp_SearchTurnLeftSlow)   ;
	UNUSED(bsp_IsCharging);
	UNUSED(bsp_IsChargeDone);
	
	UNUSED(bsp_SearchRunStraightFastBack);
	UNUSED(bsp_SearchRunStraightSlowBack);
	UNUSED(bsp_SearchTurnRightFastBack);
	UNUSED(bsp_SearchTurnRightSlowBack);
	UNUSED(bsp_SearchTurnLeftFastBack);
	UNUSED(bsp_SearchTurnLeftSlowBack);
	
	UNUSED(bsp_RotateCCW);
	
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
	
	if(isCleanRunning())
		return;
	
	if(bsp_IsTouchChargePile())
	{
		/*���ſ�ʼ���*/
		if(search.isNeedPlaySong && (xTaskGetTickCount() - search.isNeedPlaySongTick >= 1000) ) /*���ʱ���жϱ����˶������ſ�ʼ���*/
		{
			search.isNeedPlaySongTick = UINT32_T_MAX; /*�������ʱ��̶ȣ��´���Ȼ��������*/
			bsp_SperkerPlay(Song22);
			search.isNeedPlaySong = false;
		}

		if(bsp_IsCharging())
		{
			bsp_SetLedState(AT_CHARGING);
			search.lastIsChargingTick = xTaskGetTickCount();
		}
		else
		{
			if(xTaskGetTickCount() - search.lastIsChargingTick >= 1000) /*���ʱ���жϱ����˻Ƶƣ��̵���˸*/
			{
				bsp_SetLedState(AT_CHARGE_DONE);
			}
		}
		
		search.lastIsTouchTick = xTaskGetTickCount();
	}
	else /*��׮״̬��Ҫ������ģ����ǵ���Ҫ�Ȼ��������Ȼ�ᶶ��*/
	{
		//DEBUG("�Ӵ�׮:%s �����:%s ����:%s\r\n",bsp_IsTouchChargePile()?"true":"false",bsp_IsCharging()?"true":"false",bsp_IsChargeDone()?"true":"false");
		
		search.isNeedPlaySong = true;
		search.isNeedPlaySongTick = xTaskGetTickCount();
		
		if(xTaskGetTickCount() - search.lastIsTouchTick >= 500)
		{
			 /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!������Ҫ����ɨ��������*/
			if( bsp_GetLedAppState() != AT_CLEAN && 
				bsp_GetLedAppState() != AT_SEARCH_CHARGE &&
				bsp_GetLedAppState() != THREE_WHITE_TOOGLE &&
			    bsp_GetLedAppState() != AT_LINK)
			{
				bsp_SetLedState(THREE_WHITE_ON);
			}

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
			bsp_SetMotorSpeed(MotorLeft, 8);
			bsp_SetMotorSpeed(MotorRight,8);
			
			search.action++;
		}break;
		
		case 1:
		{
			/*�����ж���ײ*/
			Collision ret = bsp_CollisionScan();
			
			if(ret != CollisionNone || bsp_CliffIsDangerous(CliffLeft) || bsp_CliffIsDangerous(CliffMiddle) || bsp_CliffIsDangerous(CliffRight))
			{
				/*������������˾ͺ��ˣ��ں��˵Ĺ�����������������*/
				bsp_GoBackward();
				
				search.delay = xTaskGetTickCount();
				search.action++;
			}
			/*ǰ��2���������յ����ҷ���*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)
				&& bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
			{
				bsp_SearchRunStraightSlow();
			}
			/*ǰ��2�������ո�*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SearchRunStraightSlow();
			}
			
			
			/*1�����յ�2�� ,2�����յ����*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SearchRunStraightSlow();
			}
			
			/*2�����յ�2�� ,1�����յ��ұ�*/
			else if(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
			{
				bsp_SearchRunStraightSlow();
			}
			
			
			/*1��2�Ŷ����յ���ߣ��������յ��ұ�*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && 
				!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
			{
				bsp_SearchTurnRightSlow();
			}
			/*1��2�Ŷ����յ��ұߣ��������յ����*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && 
				!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SearchTurnLeftSlow();
			}

			
			/*1�Ų���ͬʱ�յ����ҷ��䣬2��ͬʱ�յ����ҷ���*/
			else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SearchTurnLeftSlow();
			}
			/*1����ͬʱ�յ����ҷ��䣬2����ͬʱ�յ����ҷ���*/
			else if((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SearchTurnRightSlow();
			}
			/*����4�����յ���Ǻ�����ң�����1,2�������յ��κ�,ԭ����ת*/
			else if(bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)))
			{
				bsp_PirouetteCW();
			}
			/*����3�����յ���Ǻ�����ң�����1,2�������յ��κ�,ԭ����ת*/
			else if(bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT)))
			{
				bsp_PirouetteCCW();
			}
			/*1��2������ͬʱ�յ����ҷ���*/
			else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft, 8);
				bsp_SetMotorSpeed(MotorRight,8);
			}
			
			
		}break;
		
		case 2:
		{
			/*������һ��ײ����*/
			if(xTaskGetTickCount() - search.delay >= 2500)
			{
				search.action = 1 ;
				
				if(!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH1,IR_TX_SITE_CENTER)
				&& !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_CENTER))
				{
					bsp_RotateCCW();
					search.delay = xTaskGetTickCount();
					search.action  = 3;
				}
				
				
			}
			else
			{
				if(xTaskGetTickCount() - search.delay <= 500)
				{
					bsp_GoBackward();
				}
				/*ǰ��2���������յ����ҷ���*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)
				&& bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
				{
					//bsp_SearchRunStraightSlow();
					
					bsp_SetMotorSpeed(MotorLeft, ROTATE_CCW_SPEED_L);
					bsp_SetMotorSpeed(MotorRight,ROTATE_CCW_SPEED_R);
					vTaskDelay(CORRECTION_ANGLE_TIME);	
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					
					search.action = 1 ;
				}
				/*ǰ��2�������ո�*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					//bsp_SearchRunStraightSlow();
					
					bsp_SetMotorSpeed(MotorLeft, ROTATE_CCW_SPEED_L);
					bsp_SetMotorSpeed(MotorRight,ROTATE_CCW_SPEED_R);
					vTaskDelay(CORRECTION_ANGLE_TIME);	
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					
					search.action = 1 ;
				}
				
				
				/*1�����յ�2�� ,2�����յ����*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SearchRunStraightSlowBack();
				}
				
				/*2�����յ�2�� ,1�����յ��ұ�*/
				else if(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				{
					bsp_SearchRunStraightSlowBack();
				}
				
				
				/*1��2�Ŷ����յ���ߣ��������յ��ұ�*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && 
					!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
				{
					bsp_SearchTurnLeftSlowBack();
				}
				/*1��2�Ŷ����յ��ұߣ��������յ����*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && 
					!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SearchTurnRightSlowBack();
				}

				
				/*1�Ų���ͬʱ�յ����ҷ��䣬2��ͬʱ�յ����ҷ���*/
				else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SearchTurnRightSlowBack();
				}
				/*1����ͬʱ�յ����ҷ��䣬2����ͬʱ�յ����ҷ���*/
				else if((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SearchTurnLeftSlowBack();
				}
				
				
				/*����4�����յ���Ǻ�����ң�����1,2�������յ��κ�,ԭ����ת*/
				else if(bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)))
				{
					bsp_PirouetteCW();
				}
				/*����3�����յ���Ǻ�����ң�����1,2�������յ��κ�,ԭ����ת*/
				else if(bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT)))
				{
					bsp_PirouetteCCW();
				}
				/*1��2������ͬʱ�յ����ҷ���*/
				else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SearchRunStraightSlowBack();
				}
			}
		}break;
		
		
		case 3:/*����û�к����ź���ת��*/
		{
			if(xTaskGetTickCount() - search.delay >= 800)
			{
				search.action = 1 ;
			}
		}break;
	}
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchRunStraightFast
*	����˵��: ����ֱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchRunStraightFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, STRAIGHT_SPEED_FAST);
	bsp_SetMotorSpeed(MotorRight,STRAIGHT_SPEED_FAST);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchRunStraightSlow
*	����˵��: ����ֱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchRunStraightSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, STRAIGHT_SPEED_SLOW);
	bsp_SetMotorSpeed(MotorRight,STRAIGHT_SPEED_SLOW);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchTurnRightFast
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchTurnRightFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_RIGHT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,TURN_RIGHT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchTurnRightSlow
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchTurnRightSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_RIGHT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,TURN_RIGHT_SPEED_SLOW_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchTurnLeftFast
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchTurnLeftFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_LEFT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,TURN_LEFT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchTurnLeftSlow
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchTurnLeftSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_LEFT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,TURN_LEFT_SPEED_SLOW_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_PirouetteCW
*	����˵��: ԭ��˳ʱ����ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_PirouetteCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, PIROUETTE_SPEED);
	bsp_SetMotorSpeed(MotorRight,0);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_PirouetteCCW
*	����˵��: ԭ����ʱ����ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_PirouetteCCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, 0);
	bsp_SetMotorSpeed(MotorRight,PIROUETTE_SPEED);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GoBackward
*	����˵��: �����ϰ���󣬿��ٵ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_GoBackward(void)
{
	bsp_SetMotorSpeed(MotorLeft, BACKWARD_SPEED);
	bsp_SetMotorSpeed(MotorRight,BACKWARD_SPEED);
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchRunStraightFastBack
*	����˵��: ����ֱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchRunStraightFastBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -STRAIGHT_SPEED_FAST);
	bsp_SetMotorSpeed(MotorRight,-STRAIGHT_SPEED_FAST);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchRunStraightSlowBack
*	����˵��: ����ֱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchRunStraightSlowBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -STRAIGHT_SPEED_SLOW);
	bsp_SetMotorSpeed(MotorRight,-STRAIGHT_SPEED_SLOW);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchTurnRightFastBack
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchTurnRightFastBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -TURN_RIGHT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,-TURN_RIGHT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchTurnRightSlowBack
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchTurnRightSlowBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -TURN_RIGHT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,-TURN_RIGHT_SPEED_SLOW_R);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchTurnLeftFastBack
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchTurnLeftFastBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -TURN_LEFT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,-TURN_LEFT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SearchTurnLeftSlowBack
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SearchTurnLeftSlowBack(void)
{
	bsp_SetMotorSpeed(MotorLeft, -TURN_LEFT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,-TURN_LEFT_SPEED_SLOW_R);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_RotateCCW
*	����˵��: ԭ����ת�������ֶ�������ʱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_RotateCCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, ROTATE_CCW_SPEED_L);
	bsp_SetMotorSpeed(MotorRight,ROTATE_CCW_SPEED_R);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitIO
*	����˵��: ��ʼ�������ص�IO
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitIO(void)
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

