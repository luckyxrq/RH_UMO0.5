#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1

#define MAX_SEARCH_TICK         (1000*60*5)



#define CCW_180_PULSE      1700                /*ת��180�ȵ�������*/
#define CCW_360_PULSE      (CCW_180_PULSE*2)   /*ת��360�ȵ�������*/
#define CCW_90_PULSE       (CCW_180_PULSE/2)   /*ת��90 �ȵ�������*/
#define CCW_45_PULSE       (CCW_180_PULSE/4)   /*ת��45 �ȵ�������*/


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
                      �����յ��˹��  ���ұ��ϲ���ͬʱ�յ�����խ���ź�  ����ǰ���ղ����κ�խ���ź�
**********************************************************************************************************
*/
#define ROTATE_CW_LITTLE    ( !(IR_SR_L && IR_SR_R) && IR_SR_M && F_NO_NARROW_SIGNAL)
#define ROTATE_CCW_LITTLE   ( !(IR_SL_L && IR_SL_R) && IR_SL_M && F_NO_NARROW_SIGNAL)

/*
**********************************************************************************************************
                          �����յ���2��խ��  ��Ҫԭ�ش�ת��  ����ǰ���ղ���խ���ź�
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

#define ALL_NO_SIGNAL         (SL_NO_SIGNAL && SR_NO_SIGNAL && FL_NO_SIGNAL && FR_NO_SIGNAL)

#define ONLY_F_RX_WIDE        ( (IR_FL_M ||  IR_FR_M) && F_NO_NARROW_SIGNAL && SL_NO_SIGNAL && SR_NO_SIGNAL)   

/*
**********************************************************************************************************
                                               ״̬
**********************************************************************************************************
*/
#define  SET_CW_CCW_LITTLE_FALSE()      {search.isInCW_LITTLE  = false; search.isInCCW_LITTLE = false;}

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
	uint32_t lastReallyChargeTime;
	
	bool isNeedFirstRunRandom;
	Collision collision;
	
	uint32_t pulse;
	float angle;
	uint32_t lastSIGNAL_Tick;
	uint32_t ONLY_F_RX_WIDE_CNT;
	
	bool isInBack;                    /*���˹�����*/
	bool isNeedRote180;               /*��û���κ��źŵ�����£�������Ҫ��ͷ180��*/
	uint32_t isNeedRote180StartPulse; /*��û���κ��źŵ�����£�������Ҫ��ͷ180�ȣ���ʼ����*/
	bool isBanOnlyF_Wide;
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
	search.lastSIGNAL_Tick = xTaskGetTickCount();
	search.lastReallyChargeTime = 0 ;
	search.ONLY_F_RX_WIDE_CNT = 0 ;
	search.isNeedFirstRunRandom = true;
	search.isBanOnlyF_Wide = false;
	search.isInBack = false;
	search.isNeedRote180 = false;
	search.isNeedRote180StartPulse = 0 ;
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
	search.isBanOnlyF_Wide = false;
	search.action = 0 ;
	search.delay = 0 ;
	
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
	
}	


/*
*********************************************************************************************************
*	�� �� ��: bsp_DetectIsTouchChargePile
*	����˵��: ��⵽���׮�ˣ�����ֹͣ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
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








/*
*********************************************************************************************************
*	�� �� ��: bsp_CCW_360_BY_Encoder
*	����˵��: ˳ʱ����ת360��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/










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

	/**************************************************************���������ɨ״̬ ���� �����ϴ����ݵ�״̬ ��ֱ�ӷ���**********************************************************************************/
	if(isCleanRunning() || GetCmdStartUpload())
		return;
	
	
	
	/********************************************************************��һ�������ҳ��׮�Ķ���ǰ���жϵ�********************************************************************************************/
	if(bsp_IsTouchChargePile())
	{
		/*���ſ�ʼ���*/
		if(search.isNeedPlaySong && ((xTaskGetTickCount() >= search.isNeedPlaySongTick)  && (xTaskGetTickCount() - search.isNeedPlaySongTick >= 1000)) ) /*���ʱ���жϱ����˶������ſ�ʼ���*/
		{
			search.isNeedPlaySongTick = UINT32_T_MAX; /*�������ʱ��̶ȣ��´���Ȼ��������*/
			bsp_SperkerPlay(Song22);
			search.isNeedPlaySong = false;
			
			search.lastReallyChargeTime = xTaskGetTickCount();
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
	

	/***********************************************************************ֹͣѰ�ҳ��׮**************************************************************************/

	/*ֹͣѰ��-----------��ʱ��ʱ�䵽�˻�û�ҵ����׮*/
	if(xTaskGetTickCount() - search.startTick >= MAX_SEARCH_TICK)
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_CloseAllStateRun();
		
		bsp_SetLedState(THREE_WHITE_ON); 
		bsp_SperkerPlay(Song24);
		
		return;
	}
	
	/*ֹͣѰ��-----------���Ӵ������׮��*/
	if(bsp_IsTouchChargePile() == true)
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_CloseAllStateRun();
		
		return ;
	}
	
	/********************************************************************����Ѱ�ҳ��׮�Ķ���************************************************************************/
	
	switch(search.action)
	{
		case 0:
		{
			search.pulse = bsp_GetCurrentBothPulse();
			bsp_SetMotorSpeed(MotorLeft, 3);
			bsp_SetMotorSpeed(MotorRight,-3);
			
			++search.action;
		}break;
		
		case 1: /*��1Ȧ�����У������׽������ȷ�����λ��*/
		{
			if(bsp_GetCurrentBothPulse() - search.pulse <= CCW_360_PULSE) 
			{
				if(ROTATE_CW)
				{
					bsp_SetMotorSpeed(MotorLeft, 2);
					bsp_SetMotorSpeed(MotorRight,-2);
					
					search.action = 3;
				}
				else if(ROTATE_CCW)
				{
					bsp_SetMotorSpeed(MotorLeft, -2);
					bsp_SetMotorSpeed(MotorRight,2);
					
					search.action = 3;
				}
				else if(RUN_STRAIGHT_0 ||  RUN_STRAIGHT_1)
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,3);
					
					search.action = 3;
				}
				else if(INCLINATION_GO_L_0 || INCLINATION_GO_L_1 || INCLINATION_GO_L_2)
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,5);
					
					search.action = 3;
				}
				else if(INCLINATION_GO_R_0 || INCLINATION_GO_R_1 || INCLINATION_GO_R_2)
				{
					bsp_SetMotorSpeed(MotorLeft, 5);
					bsp_SetMotorSpeed(MotorRight,3);
					
					search.action = 3;
				}
			}
			else
			{
				search.pulse = bsp_GetCurrentBothPulse();
				bsp_SetMotorSpeed(MotorLeft, -3);
				bsp_SetMotorSpeed(MotorRight, 3);
				
				++search.action;
			}
		}break;
		
		case 2:  /*��2Ȧ�����У��Ҳ�����ȷ��λ�ã���ô���˶������*/
		{
			if(bsp_GetCurrentBothPulse() - search.pulse <= CCW_360_PULSE) /*��1Ȧ�����У������׽������ȷ�����λ��*/
			{
				if(ROTATE_CW)
				{
					bsp_SetMotorSpeed(MotorLeft, 2);
					bsp_SetMotorSpeed(MotorRight,-2);
					
					search.action = 3;
				}
				else if(ROTATE_CCW)
				{
					bsp_SetMotorSpeed(MotorLeft, -2);
					bsp_SetMotorSpeed(MotorRight,2);
					
					search.action = 3;;
				}
				else if(RUN_STRAIGHT_0 ||  RUN_STRAIGHT_1)
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,3);
					
					search.action = 3;
				}
				else if(INCLINATION_GO_L_0 || INCLINATION_GO_L_1 || INCLINATION_GO_L_2)
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,5);
					
					search.action = 3;
				}
				else if(INCLINATION_GO_R_0 || INCLINATION_GO_R_1 || INCLINATION_GO_R_2)
				{
					bsp_SetMotorSpeed(MotorLeft, 5);
					bsp_SetMotorSpeed(MotorRight,3);
					
					search.action = 3;
				}
				else if(ROTATE_CW_LITTLE)
				{
					bsp_SetMotorSpeed(MotorLeft, 7);
					bsp_SetMotorSpeed(MotorRight,2);
					
					search.action = 3;
				}
				else if(ROTATE_CW_LITTLE)
				{
					bsp_SetMotorSpeed(MotorLeft, 2);
					bsp_SetMotorSpeed(MotorRight,7);
					
					search.action = 3;
				}
			}
			else /*ת�ڶ�Ȧ��ʱ�䳬���ˣ���û��֪����ô�ߣ���ֱ��*/
			{
				search.pulse = bsp_GetCurrentBothPulse();
				bsp_SetMotorSpeed(MotorLeft,  3);
				bsp_SetMotorSpeed(MotorRight, 3);
				
				++search.action;
			}
		}break;	
		
		case 3: /*���ݽǶȵ���*/
		{
			search.collision = bsp_CollisionScan();
		
			/*���ź���ײ*/
			if(ALL_NO_SIGNAL && (search.collision != CollisionNone || (bsp_CliffIsDangerous(CliffLeft) || bsp_CliffIsDangerous(CliffMiddle) || bsp_CliffIsDangerous(CliffRight))))
			{
				/*������������˾ͺ��ˣ��ں��˵Ĺ�����������������*/
				bsp_SetMotorSpeed(MotorLeft, -8);
				bsp_SetMotorSpeed(MotorRight,-8);
				
				search.isInBack = true;
				search.pulse = bsp_GetCurrentBothPulse();
				search.action = 3;
			}
			/*���ź���ײ*/
			else if(!ALL_NO_SIGNAL && (search.collision != CollisionNone || (bsp_CliffIsDangerous(CliffLeft) || bsp_CliffIsDangerous(CliffMiddle) || bsp_CliffIsDangerous(CliffRight))))
			{
				/*������������˾ͺ��ˣ��ں��˵Ĺ�����������������*/
				bsp_SetMotorSpeed(MotorLeft, -8);
				bsp_SetMotorSpeed(MotorRight,-8);
				
				search.isInBack = true;
				search.pulse = bsp_GetCurrentBothPulse();
				search.action = 3;
			}
			else if(search.isInBack)
			{
				if(bsp_GetCurrentBothPulse() - search.pulse >= _SEARCH_PILE_GO_BACK_PULSE*50)
				{
					search.isInBack = false;
					if(search.collision == CollisionLeft)
					{
						bsp_SetMotorSpeed(MotorLeft, 5);
						bsp_SetMotorSpeed(MotorRight,2);
					}
					else if(search.collision == CollisionRight)
					{
						bsp_SetMotorSpeed(MotorLeft, 2);
						bsp_SetMotorSpeed(MotorRight,5);
					}
					else
					{
						bsp_SetMotorSpeed(MotorLeft, 3);
						bsp_SetMotorSpeed(MotorRight,3);
					}
					
				}
			}
			else if(ROTATE_CW)
			{
				search.isNeedRote180 = true;
				search.isNeedRote180StartPulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 2);
				bsp_SetMotorSpeed(MotorRight,-2);
				
				search.action = 3;
			}
			else if(ROTATE_CCW)
			{
				search.isNeedRote180 = true;
				search.isNeedRote180StartPulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -2);
				bsp_SetMotorSpeed(MotorRight,2);
				
				search.action = 3;;
			}
			else if(RUN_STRAIGHT_0 ||  RUN_STRAIGHT_1)
			{
				bsp_SetMotorSpeed(MotorLeft, 3);
				bsp_SetMotorSpeed(MotorRight,3);
				
				search.action = 3;
			}
			else if(INCLINATION_GO_L_0 || INCLINATION_GO_L_1 || INCLINATION_GO_L_2)
			{
				bsp_SetMotorSpeed(MotorLeft, 3);
				bsp_SetMotorSpeed(MotorRight,5);
				
				search.action = 3;
			}
			else if(INCLINATION_GO_R_0 || INCLINATION_GO_R_1 || INCLINATION_GO_R_2)
			{
				bsp_SetMotorSpeed(MotorLeft, 5);
				bsp_SetMotorSpeed(MotorRight,3);
				
				search.action = 3;
			}
			else if(ROTATE_CW_LITTLE)
			{
				search.isNeedRote180 = true;
				search.isNeedRote180StartPulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 7);
				bsp_SetMotorSpeed(MotorRight,2);
				
				search.action = 3;
			}
			else if(ROTATE_CCW_LITTLE)
			{
				search.isNeedRote180 = true;
				search.isNeedRote180StartPulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 2);
				bsp_SetMotorSpeed(MotorRight,7);
				
				search.action = 3;
			}
			else if( search.isNeedRote180 && ALL_NO_SIGNAL && (bsp_GetCurrentBothPulse()-search.isNeedRote180StartPulse>=CCW_180_PULSE)) /*ʲô�ź�Ҳû���ˣ���ת180�Ȱ�*/
			{
				search.isNeedRote180 = false;
				
				search.pulse = bsp_GetCurrentBothPulse();
				bsp_SetMotorSpeed(MotorLeft, -3);
				bsp_SetMotorSpeed(MotorRight, 3);
				++search.action ;
			}
		}break;
		
		case 4:
		{
			if(bsp_GetCurrentBothPulse() - search.pulse >= CCW_180_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 3);
				bsp_SetMotorSpeed(MotorRight,3);
				search.action = 3 ;
			}
		}break;
		
		
	}
}



