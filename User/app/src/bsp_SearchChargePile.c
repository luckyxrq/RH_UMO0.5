#include "bsp.h"


#define RCC_ALL_LED 	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_CHARGE_TOUCH_PILE  GPIOF
#define GPIO_PIN_CHARGE_TOUCH_PILE	 GPIO_Pin_7

#define GPIO_PORT_CHARGE_IS_CHARGING  GPIOG
#define GPIO_PIN_CHARGE_IS_CHARGING	  GPIO_Pin_0

#define GPIO_PORT_CHARGE_IS_DONE      GPIOG
#define GPIO_PIN_CHARGE_IS_DONE	      GPIO_Pin_1

#define MAX_SEARCH_TICK         (1000*60*5)

#define REAL_ANGLE()      (bsp_AngleReadRaw()*0.01F)

#define _SEARCH_PILE_GO_BACK_PULSE                  (10/(3.14F*70)*1024)

/*
**********************************************************************************************************
                                            直走
**********************************************************************************************************
*/
#define RUN_STRAIGHT_0      ( IR_FL_L && IR_FL_R && IR_FR_L && IR_FR_R    )  /*前面两个都能收到2个窄角信号*/
#define RUN_STRAIGHT_1      ( IR_FL_L && !IR_FL_R && !IR_FR_L && IR_FR_R  )  /*前面两个都能收到与之正对的窄角信号*/


/*
**********************************************************************************************************
                                     前面没有收到任何窄角信号
**********************************************************************************************************
*/
#define F_NO_NARROW_SIGNAL  ( !IR_FL_L && !IR_FL_R && !IR_FR_L && !IR_FR_R  ) /*前面两个都收不到任何窄角信号*/


/*
**********************************************************************************************************
                                    边上收到了广角  需要小一点的大转弯
**********************************************************************************************************
*/
#define ROTATE_CW_LITTLE    ( !(IR_SR_L && IR_SR_R) && IR_SR_M && F_NO_NARROW_SIGNAL)
#define ROTATE_CCW_LITTLE   ( !(IR_SL_L && IR_SL_R) && IR_SL_M && F_NO_NARROW_SIGNAL)

/*
**********************************************************************************************************
                                    边上收到了2个窄角  需要大转弯
**********************************************************************************************************
*/
#define ROTATE_CW           ( (IR_SR_L && IR_SR_R) && F_NO_NARROW_SIGNAL)
#define ROTATE_CCW          ( (IR_SL_L && IR_SL_R) && F_NO_NARROW_SIGNAL)

/*
**********************************************************************************************************
                                     已经几乎到了正前面，准备微调
**********************************************************************************************************
*/
#define INCLINATION_GO_L_0    ( IR_FL_L && IR_FL_R && !IR_FR_L && IR_FR_R    )  /*左能收左右，右能收右不能收左*/
#define INCLINATION_GO_L_1    ( IR_FL_R && !IR_FR_L && !IR_FR_R    )            /*左能收右，并且右不能收左也不能收右*/
#define INCLINATION_GO_L_2    ( IR_FL_L && IR_FL_R && !IR_FR_L && !IR_FR_R    ) /*左能同时左右 右既不能左也不能右*/

#define INCLINATION_GO_R_0    ( IR_FL_L && !IR_FL_R && IR_FR_L && IR_FR_R    )  /*左能左  左不能右 并且 右能左也能右*/
#define INCLINATION_GO_R_1    ( !IR_FL_L && !IR_FL_R && IR_FR_L              )  /*左既不能左也不能右  右能左*/
#define INCLINATION_GO_R_2    ( !IR_FL_L && !IR_FL_R && IR_FR_L && IR_FR_R   )  /*左既不能左也不能右，右能同时左右*/

/*
**********************************************************************************************************
                  头部收到了广角但是没有收到窄角，两边没有任何信号(科沃斯原地左转，再向右画弧线)
**********************************************************************************************************
*/
#define SL_NO_SIGNAL          (!IR_SL_L && !IR_SL_R && !IR_SL_M)  /*边上的左边 没有任何信号*/
#define SR_NO_SIGNAL          (!IR_SR_L && !IR_SR_R && !IR_SR_M)  /*边上的右边 没有任何信号*/

#define FL_NO_SIGNAL          (!IR_FL_L && !IR_FL_R && !IR_FL_M)  /*前面左边没有任何信号*/
#define FR_NO_SIGNAL          (!IR_FR_L && !IR_FR_R && !IR_FR_M)  /*前面右边没有任何信号*/

#define ALL_NO_SIGNAL         (SL_NO_SIGNAL && SR_NO_SIGNAL && FL_NO_SIGNAL && FR_NO_SIGNAL)

#define ONLY_F_RX_WIDE        ( (IR_FL_M ||  IR_FR_M) && F_NO_NARROW_SIGNAL && SL_NO_SIGNAL && SR_NO_SIGNAL)   

/*
**********************************************************************************************************
                                               状态
**********************************************************************************************************
*/
#define  SET_CW_CCW_LITTLE_FALSE()      {search.isInCW_LITTLE  = false; search.isInCCW_LITTLE = false;}

typedef enum
{
	eNone = 0 ,          /*没有碰撞*/
	eHasSignalCollision, /*和充电桩碰上了后碰撞*/
	eNoSignalCollision   /*没有信号的碰撞，没有碰上充电桩，比如碰上了墙壁*/
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
	uint32_t lastSIGNAL_Tick;
	uint32_t ONLY_F_RX_WIDE_CNT;
	
	bool isInCW_LITTLE;  /*顺时针旋转的时候收到了前面的信号*/
	bool isInCCW_LITTLE; /*逆时针旋转的时候收到了前面的信号*/
	
	bool isInCW_LITTLE_RX_F;  /*顺时针旋转的时候收到了前面的信号*/
	bool isInCCW_LITTLE_RX_F; /*逆时针旋转的时候收到了前面的信号*/
}Serach;


static Serach search;

/*
*********************************************************************************************************
*	函 数 名: bsp_StartSearchChargePile
*	功能说明: 开启寻找充电桩状态机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartSearchChargePile(void)
{
	
	
	search.action = 0 ;
	search.delay = 0 ;
	search.startTick = xTaskGetTickCount();
	search.lastSIGNAL_Tick = xTaskGetTickCount();
	search.ONLY_F_RX_WIDE_CNT = 0 ;
	search.isNeedFirstRunRandom = true;
	search.isRunning = true;
	
	/*防止编译器警告*/

	bsp_IRD_StartWork();
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StopSearchChargePile
*	功能说明: 停止寻找充电桩状态机
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: bsp_SearchChargePile
*	功能说明: 寻找充电桩状态机
*	形    参: 无
*	返 回 值: 无
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
		/*播放开始充电*/
		if(search.isNeedPlaySong && (xTaskGetTickCount() >= search.isNeedPlaySongTick  && xTaskGetTickCount() - search.isNeedPlaySongTick >= 1000) ) /*这个时间判断避免了抖动播放开始充电*/
		{
			search.isNeedPlaySongTick = UINT32_T_MAX; /*给个最大时间刻度，下次自然不会满足*/
			bsp_SperkerPlay(Song22);
			search.isNeedPlaySong = false;
		}

		if(bsp_IsChargeDone())
		{
			
			if(xTaskGetTickCount() - search.lastIsChargeDoneTick >= 1000) /*这个时间判断避免了黄灯，绿灯闪烁*/
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
	else /*离桩状态需要立马更改，但是灯需要等会儿处理，不然会抖动*/
	{
		if((xTaskGetTickCount() >= search.lastIsTouchTick)  &&  (xTaskGetTickCount() - search.lastIsTouchTick >= 500))
		{
			 /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!治理需要根据扫地与否更改*/
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
		

	/*计时，时间到了还没找到充电桩*/
	if(xTaskGetTickCount() - search.startTick >= MAX_SEARCH_TICK)
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_CloseAllStateRun();
		
		bsp_SetLedState(THREE_WHITE_ON); 
		bsp_SperkerPlay(Song24);
		
		return;
	}
	
	
	
	/*充电*/
	if(bsp_IsTouchChargePile() == true)
	{
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_CloseAllStateRun();
		
		return ;
	}
	
	/*先随机清扫*/
	if(search.isNeedFirstRunRandom)
	{
		bsp_StartStrategyRandom();
		search.isNeedFirstRunRandom = false;
	}
	
	
	
	/*如果在执行随机清扫，则不执行后面的寻找充电桩*/
	if(bsp_IsStartStrategyRandom())
	{
		if(!ALL_NO_SIGNAL) /*收到了信号就执行后面的寻找充电桩*/
		{
			DEBUG("退出随机\r\n");
			
			bsp_SetMotorSpeed(MotorLeft,0);
			bsp_SetMotorSpeed(MotorRight,0);
			bsp_StopStrategyRandom();
		}
		return;
	}
	
	
	
	if(!ALL_NO_SIGNAL) /*最后一次有信号时间*/
	{
		search.lastSIGNAL_Tick = xTaskGetTickCount();
	}
	
	
	if(xTaskGetTickCount() - search.lastSIGNAL_Tick >= 1000*60*1.5)
	{
		search.action = 0 ;
		search.lastSIGNAL_Tick = xTaskGetTickCount();
		
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_StartStrategyRandom();
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
			/*首先判断碰撞*/
			search.collision = bsp_CollisionScan();
			
			if(search.collision != CollisionNone )
			{
				/*不管如何碰到了就后退，在后退的过程中再来调节轮子*/
				bsp_SetMotorSpeed(MotorLeft, -3);
				bsp_SetMotorSpeed(MotorRight,-3);
				
				search.delay = xTaskGetTickCount();
				search.action++;
			}
			else
			{
				static bool isNeedOpenONLY_F_RX_WIDE = true;
				if(ONLY_F_RX_WIDE)
				{
					if(++search.ONLY_F_RX_WIDE_CNT >= 100 && isNeedOpenONLY_F_RX_WIDE)
					{
						isNeedOpenONLY_F_RX_WIDE = false;
						search.ONLY_F_RX_WIDE_CNT = 0 ;
						bsp_SetMotorSpeed(MotorLeft, 0);
						bsp_SetMotorSpeed(MotorRight,0);
						
						search.action = 3 ;
						return;
					}
				}
				else
				{
					search.ONLY_F_RX_WIDE_CNT = 0 ;
				}
				
				
				if(search.isInCW_LITTLE)
				{
					if((IR_FL_M || IR_FR_M) && (!IR_FL_L && !IR_FL_R && !IR_FR_L && !IR_FR_R) )
					{
						search.ONLY_F_RX_WIDE_CNT = 0 ;
						bsp_SetMotorSpeed(MotorLeft, 0);
						bsp_SetMotorSpeed(MotorRight,0);
						
						search.action = 3 ;
					}
				}
				
				
				if(ROTATE_CW_LITTLE)
				{
					bsp_SetMotorSpeed(MotorLeft, 7);
					bsp_SetMotorSpeed(MotorRight,2);
					
					search.isInCW_LITTLE = true;
				}
				else if(ROTATE_CW_LITTLE)
				{
					bsp_SetMotorSpeed(MotorLeft, 2);
					bsp_SetMotorSpeed(MotorRight,7);
					search.isInCCW_LITTLE = true;
				}
				else if(ROTATE_CW)
				{
					bsp_SetMotorSpeed(MotorLeft, 2);
					bsp_SetMotorSpeed(MotorRight,-2);
					
					SET_CW_CCW_LITTLE_FALSE();
				}
				else if(ROTATE_CCW)
				{
					bsp_SetMotorSpeed(MotorLeft, -2);
					bsp_SetMotorSpeed(MotorRight,2);
					
					SET_CW_CCW_LITTLE_FALSE();
				}
				else if(RUN_STRAIGHT_0 ||  RUN_STRAIGHT_1)
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,3);
					
					SET_CW_CCW_LITTLE_FALSE();
				}
				else if(INCLINATION_GO_L_0 || INCLINATION_GO_L_1 || INCLINATION_GO_L_2)
				{
					bsp_SetMotorSpeed(MotorLeft, 3);
					bsp_SetMotorSpeed(MotorRight,5);
					
					SET_CW_CCW_LITTLE_FALSE();
				}
				else if(INCLINATION_GO_R_0 || INCLINATION_GO_R_1 || INCLINATION_GO_R_2)
				{
					bsp_SetMotorSpeed(MotorLeft, 5);
					bsp_SetMotorSpeed(MotorRight,3);
					
					SET_CW_CCW_LITTLE_FALSE();
				}
				else if(F_NO_NARROW_SIGNAL) 
				{
//					bsp_SetMotorSpeed(MotorLeft, 3);
//					bsp_SetMotorSpeed(MotorRight,3);
					
					SET_CW_CCW_LITTLE_FALSE();
				}
			}
		}break;
		

		case 2:
		{
			if(xTaskGetTickCount() - search.delay >= 3900)
			{
				if(FL_NO_SIGNAL && FR_NO_SIGNAL) /*平常的碰撞处理*/
				{
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					
					search.action = 10 ;
				}
				else
				{
					search.action = 0 ;
				}
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
			//if(xTaskGetTickCount() - search.delay >= 3600  || bsp_CollisionScan() != CollisionNone)
			if(bsp_CollisionScan() != CollisionNone)
			{
				
				if(SL_NO_SIGNAL && FL_NO_SIGNAL && FR_NO_SIGNAL)  /*边是错的，则换边*/
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
				bsp_SetMotorSpeed(MotorRight,7);
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
		
		case 10: /*从这里开始处理碰撞往复问题*/
		{
			search.angle = REAL_ANGLE();
			bsp_SetMotorSpeed(MotorLeft, -3);
			bsp_SetMotorSpeed(MotorRight,3);
			++search.action;
		}break;
		
		case 11:
		{
			if(ABS(REAL_ANGLE() - bsp_AngleAdd(search.angle, 60)) <= 10)
			{
				bsp_SetMotorSpeed(MotorLeft, 3);
				bsp_SetMotorSpeed(MotorRight,3);
				search.delay = xTaskGetTickCount();
				search.action = 1 ;
			}
		}break;
	}
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitIO
*	功能说明: 初始化充电相关的IO
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitChargeIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	
	/*上桩检测*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CHARGE_TOUCH_PILE;
	GPIO_Init(GPIO_PORT_CHARGE_TOUCH_PILE, &GPIO_InitStructure);
	
	/*正在充电中*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CHARGE_IS_CHARGING;
	GPIO_Init(GPIO_PORT_CHARGE_IS_CHARGING, &GPIO_InitStructure);
	
	/*充电完成*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CHARGE_IS_DONE;
	GPIO_Init(GPIO_PORT_CHARGE_IS_DONE, &GPIO_InitStructure);
	
}

	
/*
*********************************************************************************************************
*	函 数 名: bsp_IsTouchChargePile
*	功能说明: 获取充电反馈
*	形    参：无
*	返 回 值: 无
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
*	函 数 名: bsp_IsCharging
*	功能说明: 是否已经充满
*	形    参：无
*	返 回 值: 无
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
*	函 数 名: bsp_IsChargeDone
*	功能说明: 是否已经充满
*	形    参：无
*	返 回 值: 无
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

