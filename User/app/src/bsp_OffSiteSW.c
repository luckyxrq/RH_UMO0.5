#include "bsp.h"

#define RCC_ALL_OFFSITE_SW 	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE)

#define GPIO_PORT_OFFSITE_SW_L  GPIOA
#define GPIO_PIN_OFFSITE_SW_L   GPIO_Pin_12


#define GPIO_PORT_OFFSITE_SW_R  GPIOE
#define GPIO_PIN_OFFSITE_SW_R   GPIO_Pin_12


/*
*********************************************************************************************************
*	函 数 名: bsp_InitOffSiteSW
*	功能说明: 初始化离地开关使能引脚
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitOffSiteSW(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_OFFSITE_SW, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	/* 浮空输入 */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_OFFSITE_SW_L;
	GPIO_Init(GPIO_PORT_OFFSITE_SW_L, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	/* 浮空输入 */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_OFFSITE_SW_R;
	GPIO_Init(GPIO_PORT_OFFSITE_SW_R, &GPIO_InitStructure);

}


/*
*********************************************************************************************************
*	函 数 名: bsp_OffSiteGetState
*	功能说明: 返回当前离地开关状态
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: bsp_OffSiteProc
*	功能说明: 离地开关处理函数
*	形    参: 无
*	返 回 值: 无
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
				/*语音报警*/
				bsp_SperkerPlay(Song16);

				DEBUG("离地开关\r\n");
	
				/*灯光恢复最开始*/
				bsp_LedOn(LED_LOGO_CLEAN);
				bsp_LedOn(LED_LOGO_POWER);
				bsp_LedOn(LED_LOGO_CHARGE);
				bsp_LedOff(LED_COLOR_YELLOW);
				bsp_LedOff(LED_COLOR_GREEN);
				bsp_LedOff(LED_COLOR_RED);
				
				bsp_StopRunToggleLED();
				
				/*复位上一次的按键状态*/
				bsp_SetKeyRunLastState(RUN_STATE_DEFAULT);
				
				/*关闭各种状态机*/
				bsp_StopSearchChargePile();
				bsp_StopCliffTest();
				bsp_StartUpdateCleanStrategyB();
				bsp_StartVacuum();
				bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
				bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
				
				offSiteProc.action++;
			}
		}break;
		
		case 1: /*等待主机不悬空*/
		{
			state = bsp_OffSiteGetState();
			if(state == OffSiteNone)
			{
				offSiteProc.action = 0 ;
			}
		}break;
	}
}

