#include "bsp.h"

#define POWER_ON_LED_TIME_INTERVAL            500
#define POWER_ON_LED_MAX_TIMES                5
#define POWER_ON_LED_ON_CONSTANT_MAX_TIME     10*60*1000         

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
static RunControl runControl;
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
					bsp_StartRunControl();    /*���������߼�����*/
					
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


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitRunControl
*	����˵��: ��ʼ������״̬
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitRunControl(void)
{
	/*״̬���ṹ*/
	runControl.isRunnng = false;
	runControl.action = 0 ;
	runControl.delay = 0 ;
	
	/*����״̬��ʼ��*/
	runControl.lastState = RUN_STATE_DEFAULT;
	
	/*����ֵ��ʼ��*/
	runControl.isHomeKey = false;
	runControl.isPowerKey = false;
	runControl.isChargeKey = false;
	runControl.isCleanKey = false;
	
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_SetHomeKey
*	����˵��: Home�����̰�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetHomeKey(bool val)
{
	runControl.isHomeKey = val;

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetPowerKey
*	����˵��: Home�����������ϵ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetPowerKey(bool val)
{
	runControl.isPowerKey = val;

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetChargeKey
*	����˵��: ��簴���̰�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetChargeKey(bool val)
{
	runControl.isChargeKey = val;

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetCleanKey
*	����˵��: ��ɨ�����̰�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetCleanKey(bool val)
{
	runControl.isCleanKey = val;

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetSuspendKey
*	����˵��: �����������̰�������Ӧ���߼�������ͣ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetSuspendKey(bool val)
{
	runControl.isSuspendKey = val;

}


/*
*********************************************************************************************************
*	�� �� ��: bsp_StartRunControl
*	����˵��: ������������״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartRunControl(void)
{
	/*״̬���ṹ*/
	runControl.action = 0 ;
	runControl.delay = 0 ;
	
	/*����״̬��ʼ��*/
	runControl.lastState = RUN_STATE_DEFAULT;
	
	/*����ֵ��ʼ��*/
	runControl.isHomeKey = false;
	runControl.isPowerKey = false;
	runControl.isChargeKey = false;
	runControl.isCleanKey = false;
	
	runControl.isRunnng = true;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_StopRunControl
*	����˵��: �رհ�������״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StopRunControl(void)
{
	/*״̬���ṹ*/
	runControl.isRunnng = false;
	runControl.action = 0 ;
	runControl.delay = 0 ;
	
	/*����״̬��ʼ��*/
	runControl.lastState = RUN_STATE_DEFAULT;

	
	/*����ֵ��ʼ��*/
	runControl.isHomeKey = false;
	runControl.isPowerKey = false;
	runControl.isChargeKey = false;
	runControl.isCleanKey = false;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_RunControl
*	����˵��: ��������״̬�л����ȣ������Ա�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_RunControl(void)
{
	if(!runControl.isRunnng)
		return;
	
	switch(runControl.action)
	{
		case 0:
		{
			/*HOME����������*/
			if(runControl.isHomeKey)
			{
				runControl.isHomeKey = false;
				runControl.lastState = RUN_STATE_HOME;
				bsp_SperkerPlay(Song3);
				
				runControl.action++;
			}
			/*Charge����������*/
			else if(runControl.isChargeKey)
			{
				runControl.isChargeKey = false;
				runControl.lastState = RUN_STATE_CHARGE;
				bsp_SperkerPlay(Song5);
				
				runControl.action++;
			}
			/*Clean����������*/
			else if(runControl.isCleanKey)
			{
				runControl.isCleanKey = false;
				runControl.lastState = RUN_STATE_CLEAN;
				bsp_SperkerPlay(Song3);
				
				runControl.action++;
			}
			/*Power����������*/
			else if(runControl.isPowerKey)
			{
				runControl.isPowerKey = false;
				bsp_SperkerPlay(Song2);
				
				runControl.action++;
			}
			
		}break;
		
		case 1:
		{
			if(runControl.isSuspendKey)
			{
				runControl.isSuspendKey = false;
				
				if(runControl.lastState == RUN_STATE_HOME)
				{
					bsp_SperkerPlay(Song4);
				}
				else if(runControl.lastState == RUN_STATE_CHARGE)
				{
					
				}
				else if(runControl.lastState == RUN_STATE_CLEAN)
				{
					bsp_SperkerPlay(Song4);
				}
				
				runControl.action = 0 ;
			}
		}break;
	}
    	
	

}



