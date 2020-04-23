#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1


#define CH1_HAVE_ANG_SIGNAL   (bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT)||bsp_IR_GetRev(IR_CH1,IR_TX_SITE_CENTER)||bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
#define CH2_HAVE_ANG_SIGNAL   (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT)||bsp_IR_GetRev(IR_CH2,IR_TX_SITE_CENTER)||bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
#define CH3_HAVE_ANG_SIGNAL   (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT)||bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER)||bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT))
#define CH4_HAVE_ANG_SIGNAL   (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT)||bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER)||bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)) 
		     
/*Ѱ�ҳ��ׯ֮ǰ����ת����������תֹͣ��ԭ��*/
#define CCW_STOP_BY_CH1_2      1
#define CCW_STOP_BY_CH3        2
#define CCW_STOP_BY_CH4        3
#define CCW_STOP_BY_OUTTIME    4

typedef struct
{
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
	
	float angle;
	
	bool isNeedPlaySong;
	uint32_t isNeedPlaySongTick;
	uint32_t lastIsChargingTick;
	uint32_t lastIsTouchTick;
	uint32_t lastIsNeedEdgeTick;
	
	
	uint8_t ccwSearchStopBy;
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
	
	//bsp_IRD_StopWork();
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
		bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
		bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));

		bsp_SperkerPlay(Song22);
		while(1);
	}
	
	if(bsp_IsTouchChargePile())
	{
		/*���ſ�ʼ���*/
		if(search.isNeedPlaySong && (bsp_GetRunTime() - search.isNeedPlaySongTick >= 1000) ) /*���ʱ���жϱ����˶������ſ�ʼ���*/
		{
			search.isNeedPlaySongTick = UINT32_T_MAX; /*�������ʱ��̶ȣ��´���Ȼ��������*/
			bsp_SperkerPlay(Song22);
			search.isNeedPlaySong = false;
		}

		if(bsp_IsCharging())
		{
			bsp_SetLedState(AT_CHARGING);
			search.lastIsChargingTick = bsp_GetRunTime();
			DEBUG("is charging\r\n");
		}
		else
		{
			if(bsp_GetRunTime() - search.lastIsChargingTick >= 1000) /*���ʱ���жϱ����˻Ƶƣ��̵���˸*/
			{
				bsp_SetLedState(AT_CHARGE_DONE);
			}

			DEBUG("charge done\r\n");
		}
		
		search.lastIsTouchTick = bsp_GetRunTime();
	}
	else /*��׮״̬��Ҫ������ģ����ǵ���Ҫ�Ȼ��������Ȼ�ᶶ��*/
	{
		search.isNeedPlaySong = true;
		search.isNeedPlaySongTick = bsp_GetRunTime();
		
		if(bsp_GetRunTime() - search.lastIsTouchTick >= 500)
		{
			bsp_SetLedState(THREE_WHITE_ON);   /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!������Ҫ����ɨ��������*/
			
			 search.lastIsTouchTick = UINT32_T_MAX;
		}
		
	}
	
	
	if(!search.isRunning)
		return;
	
	/*��⸽��û�г��׮�ź�*/
	if(!(CH1_HAVE_ANG_SIGNAL || CH2_HAVE_ANG_SIGNAL || CH3_HAVE_ANG_SIGNAL || CH4_HAVE_ANG_SIGNAL))
	{
		search.lastIsNeedEdgeTick = bsp_GetRunTime(); 
	}
	
	if(bsp_GetRunTime() - search.lastIsNeedEdgeTick >= 10*1000)
	{
		bsp_StopSearchChargePile();
		bsp_StartEdgewiseRun();
		return ;
	}
	
	switch(search.action)
	{
		case 0:
		{
			/*ֱ����ת*/
			bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+100));
			search.angle = bsp_AngleRead();
			search.delay = bsp_GetRunTime();
			++search.action;
		}break;
		
		case 1:
		{
			if(CH1_HAVE_ANG_SIGNAL || CH2_HAVE_ANG_SIGNAL)
			{
				search.ccwSearchStopBy = CCW_STOP_BY_CH1_2;
				
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				++search.action;
			}
			else if(CH3_HAVE_ANG_SIGNAL)
			{
				search.ccwSearchStopBy = CCW_STOP_BY_CH3;
				
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				++search.action;
			}
			else if(CH4_HAVE_ANG_SIGNAL)
			{
				search.ccwSearchStopBy = CCW_STOP_BY_CH4;
				
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				++search.action;
			}
			else if(((bsp_GetRunTime()-search.delay)>=3000 && abs(bsp_AngleRead()-search.angle)<=10.0F))
			{
				search.ccwSearchStopBy = CCW_STOP_BY_OUTTIME;
				
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				++search.action;
			}
			
			
		}break;
		
		case 2:
		{
			if(search.ccwSearchStopBy == CCW_STOP_BY_CH3) /*��ʱӦ��ת90����ȥѰ��*/
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+100));
				
				search.angle =  bsp_AngleAdd(bsp_AngleRead(),90);
				search.delay = bsp_GetRunTime();
				
				++search.action;
			}
			else
			{
				search.action = 4 ;
			}
		}break;
		
		case 3:
		{
			if(((bsp_GetRunTime()-search.delay)>=1000 && abs(bsp_AngleRead()-search.angle)<=10.0F))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				++search.action;
			}
		}break;
		
		case 4:
		{
			if(bsp_CollisionScan() != CollisionNone)
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				search.delay = bsp_GetRunTime();
				++search.action;
			}
			/*ǰ��2���������յ����ҷ���*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)
				&& bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			/*ǰ��2�������ո�*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			
			
			/*1�����յ�2�� ,2�����յ����*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			
			/*2�����յ�2�� ,1�����յ��ұ�*/
			else if(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			
			
			/*1��2�Ŷ����յ���ߣ��������յ��ұ�*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && 
				!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(220));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			/*1��2�Ŷ����յ��ұߣ��������յ����*/
			else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && 
				!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(220));
			}

			
			/*1�Ų���ͬʱ�յ����ҷ��䣬2��ͬʱ�յ����ҷ���*/
			else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(220));
			}
			/*1����ͬʱ�յ����ҷ��䣬2����ͬʱ�յ����ҷ���*/
			else if((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(220));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			/*����4�����յ���Ǻ�����ң�����1,2�������յ��κ�,ԭ����ת*/
			else if(bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			}
			/*����3�����յ���Ǻ�����ң�����1,2�������յ��κ�,ԭ����ת*/
			else if(bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
			/*1��2������ͬʱ�յ����ҷ���*/
			else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
			}
		}break;
		
		case 5:
		{
			if(bsp_GetRunTime() - search.delay <= 2000)
			{
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-200));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-200));
			}
			else
			{
				/*ǰ��2���������յ����ҷ���*/
				if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)
					&& bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
				}
				/*ǰ��2�������ո�*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
				}
				
				
				/*1�����յ�2�� ,2�����յ����*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
				}
				
				/*2�����յ�2�� ,1�����յ��ұ�*/
				else if(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
				}
				
				
				/*1��2�Ŷ����յ���ߣ��������յ��ұ�*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && 
					!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-220));
				}
				/*1��2�Ŷ����յ��ұߣ��������յ����*/
				else if(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT) && 
					!bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && !bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-220));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				}

				
				/*1�Ų���ͬʱ�յ����ҷ��䣬2��ͬʱ�յ����ҷ���*/
				else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& (bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-220));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				}
				/*1����ͬʱ�յ����ҷ��䣬2����ͬʱ�յ����ҷ���*/
				else if((bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-220));
				}
				/*����4�����յ���Ǻ�����ң�����1,2�������յ��κ�,ԭ����ת*/
				else if(bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				}
				/*����3�����յ���Ǻ�����ң�����1,2�������յ��κ�,ԭ����ת*/
				else if(bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER) && (bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT) || bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT)))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				}
				/*1��2������ͬʱ�յ����ҷ���*/
				else if(!(bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT))
					&& !(bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT) && bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)))
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));
				}
				
				if(bsp_GetRunTime() - search.delay >= 3000)
				{
					--search.action;
				}
			}
		}break;
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

