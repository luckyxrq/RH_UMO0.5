#include "bsp.h"

#define POWER_ON_LED_TIME_INTERVAL            500
#define POWER_ON_LED_MAX_TIMES                5
#define POWER_ON_LED_ON_CONSTANT_MAX_TIME     10*60*1000     

#define WIFI_SMART_CONFIG_TOGGLE_TIMES        20

typedef struct
{
	/*�ϵ���˸10S��500MS��˸һ�Σ���20��*/
	volatile bool isRunning;
	volatile uint32_t action;
	volatile uint32_t delay;
	
	/*��˸��������*/
	volatile uint8_t times;
}PowerOnToggle;

static PowerOnToggle powerOnToggle;
static bool isSelfCheckingReady = false;


/*
*********************************************************************************************************
*	�� �� ��: bsp_IsSelfCheckingReady
*	����˵��: �����Ƿ��Լ����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
bool bsp_IsSelfCheckingReady(void)
{
	return isSelfCheckingReady;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetSelfCheckingReady
*	����˵��: �����Ƿ��Լ����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetSelfCheckingReady(bool chk)
{
	isSelfCheckingReady = chk;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_StartPowerOnToggle
*	����˵��: ��������ʱ�̵ĳ�ʼ��״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartPowerOnToggle(void)
{
	powerOnToggle.action = 0 ;
	powerOnToggle.delay = 0 ;
	powerOnToggle.times = 0 ;
	powerOnToggle.isRunning = true;
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StopPowerOnToggle
*	����˵��: �رտ���ʱ�̵ĳ�ʼ��״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StopPowerOnToggle(void)
{
	powerOnToggle.isRunning = false;
	powerOnToggle.action = 0 ;
	powerOnToggle.delay = 0 ;
	powerOnToggle.times = 0 ;
	
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_PowerOnToggle
*	����˵��: ����ʱ�̵ĳ�ʼ��״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_PowerOnToggle(void)
{
	if(!powerOnToggle.isRunning)
		return;
	
	switch(powerOnToggle.action)
	{
		case 0: /*��  POWER_ON_LED_TIME_INTERVAL*/
		{
			bsp_LedOn(LED_LOGO_CLEAN);
			bsp_LedOn(LED_LOGO_POWER);
			bsp_LedOn(LED_LOGO_CHARGE);

			powerOnToggle.delay = xTaskGetTickCount();
			powerOnToggle.action++;
		}break;
		
		case 1: /*��  POWER_ON_LED_TIME_INTERVAL*/
		{
			if(xTaskGetTickCount() - powerOnToggle.delay >= POWER_ON_LED_TIME_INTERVAL)
			{
				bsp_LedOff(LED_LOGO_CLEAN);
				bsp_LedOff(LED_LOGO_POWER);
				bsp_LedOff(LED_LOGO_CHARGE);
				powerOnToggle.delay = xTaskGetTickCount();
				powerOnToggle.action++;
			}
		}break;
		
		case 2: /*��  POWER_ON_LED_TIME_INTERVAL*/
		{
			if(xTaskGetTickCount() - powerOnToggle.delay >= POWER_ON_LED_TIME_INTERVAL)
			{
				powerOnToggle.action = 0;
				
				/*��˸�Ĵ�������*/
				if(++powerOnToggle.times >=  POWER_ON_LED_MAX_TIMES)
				{
					powerOnToggle.delay = xTaskGetTickCount();
					/*����10 min*/
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					
					bsp_SetSelfCheckingReady(true);
					
					powerOnToggle.action = 3;
				}
			}
		}break;
		
		case 3: /*10  ���Ӻ�Ϩ��LED������͹���*/
		{
			if(xTaskGetTickCount() - powerOnToggle.delay >=  POWER_ON_LED_ON_CONSTANT_MAX_TIME)
			{
				bsp_LedOff(LED_LOGO_CLEAN);
				bsp_LedOff(LED_LOGO_POWER);
				bsp_LedOff(LED_LOGO_CHARGE);

				bsp_StopPowerOnToggle();
			}
			
		}break;
	}
	
	
}


typedef struct
{
	bool isRunning;
	uint32_t action;
	uint32_t delay;
	uint32_t times;
	
	LED_SN sn;
	
}RunToggleLED;

static RunToggleLED runToggleLED;

void bsp_StartRunToggleLED(LED_SN sn)
{
	runToggleLED.sn = sn;
	
	runToggleLED.action = 0 ;
	runToggleLED.delay = 0 ;
	runToggleLED.isRunning = true;

}

void bsp_StopRunToggleLED(void)
{
	runToggleLED.isRunning = false;
	runToggleLED.action = 0 ;
	runToggleLED.delay = 0 ;
	
	bsp_LedOn(LED_LOGO_CLEAN);
	bsp_LedOn(LED_LOGO_POWER);
	bsp_LedOn(LED_LOGO_CHARGE);
	bsp_LedOff(LED_COLOR_YELLOW);
	bsp_LedOff(LED_COLOR_GREEN);
	bsp_LedOff(LED_COLOR_RED);
	
}

void bsp_RunToggleLED(void)
{
	if(!runToggleLED.isRunning)
		return;
	
	switch(runToggleLED.action)
	{
		case 0:
		{
			switch(runToggleLED.sn)
			{
				case LED_LOGO_CLEAN:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}break;
				
				case LED_LOGO_POWER:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}break;
				
				case LED_LOGO_CHARGE:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}break;
				
				case LED_COLOR_YELLOW:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOff(LED_LOGO_CHARGE);
					bsp_LedOn(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}break;
				
				case LED_COLOR_GREEN:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOff(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOn(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
				}break;
				
				case LED_COLOR_RED:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOff(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOn(LED_COLOR_RED);
				}break;
				
				case LED_WIFI_LINK:
				{
					bsp_LedOn(LED_LOGO_CLEAN);
					bsp_LedOn(LED_LOGO_POWER);
					bsp_LedOn(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
					
					runToggleLED.times = WIFI_SMART_CONFIG_TOGGLE_TIMES;
				}break;
			}
			
			runToggleLED.delay = xTaskGetTickCount();
			runToggleLED.action++;
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - runToggleLED.delay >= (runToggleLED.sn == LED_WIFI_LINK ? 300 : 500))
			{
				if(runToggleLED.sn == LED_WIFI_LINK) /*��������WIFI����*/
				{
					if(runToggleLED.times >= 1)
					{
						bsp_LedToggle(LED_LOGO_CLEAN);
						bsp_LedToggle(LED_LOGO_CHARGE);
						--runToggleLED.times;
					}
					else
					{
						bsp_LedOn(LED_LOGO_CLEAN);
						bsp_LedOn(LED_LOGO_CHARGE);
					}
					
				}
				else
				{
					bsp_LedToggle(runToggleLED.sn);
				}
				
				
				runToggleLED.delay = xTaskGetTickCount();
				runToggleLED.action++;
			}
		}break;
		
		case 2:
		{
			if(xTaskGetTickCount() - runToggleLED.delay >= (runToggleLED.sn == LED_WIFI_LINK ? 300 : 500)) 
			{
				if(runToggleLED.sn == LED_WIFI_LINK) /*��������WIFI����*/
				{
					
					if(runToggleLED.times >= 1)
					{
						bsp_LedToggle(LED_LOGO_CLEAN);
						bsp_LedToggle(LED_LOGO_CHARGE);
						--runToggleLED.times;
					}
					else
					{
						bsp_LedOn(LED_LOGO_CLEAN);
						bsp_LedOn(LED_LOGO_CHARGE);
					}
				}
				else
				{
					bsp_LedToggle(runToggleLED.sn);
				}
				
				runToggleLED.delay = xTaskGetTickCount();
				runToggleLED.action = 1;
			}
			
		}break;
	}
	
}

