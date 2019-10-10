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
	runControl.lastState = ROBOT_STATE_DEFAULT;
	runControl.currentState = ROBOT_STATE_DEFAULT;
	runControl.workMethod = ROBOT_WORKWAY_DEFAULT;
	runControl.errSN = ROBOT_ERROR_NUM_DEFAULT;
	
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
	runControl.lastState = ROBOT_STATE_DEFAULT;
	runControl.currentState = ROBOT_STATE_DEFAULT;
	runControl.workMethod = ROBOT_WORKWAY_DEFAULT;
	runControl.errSN = ROBOT_ERROR_NUM_DEFAULT;
	
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
	runControl.lastState = ROBOT_STATE_DEFAULT;
	runControl.currentState = ROBOT_STATE_DEFAULT;
	runControl.workMethod = ROBOT_WORKWAY_DEFAULT;
	runControl.errSN = ROBOT_ERROR_NUM_DEFAULT;
	
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
	
	/*������һ��״̬*/
	runControl.lastState = runControl.currentState;
    	
	/*HOME����������*/
    if(runControl.isHomeKey)
    {
		runControl.isHomeKey = false;
		
        if(runControl.lastState == ROBOT_STATE_DEFAULT)       runControl.currentState = ROBOT_STATE_INIT;
        else if(runControl.lastState == ROBOT_STATE_WORKING)  runControl.currentState = ROBOT_STATE_SUSPEND;
        else if(runControl.lastState == ROBOT_STATE_CHARGING) runControl.currentState = ROBOT_STATE_SUSPEND;
        else if(runControl.lastState == ROBOT_STATE_SUSPEND)  runControl.currentState = ROBOT_STATE_WORKING;
        else if(runControl.lastState == ROBOT_STATE_STANDBY)  runControl.currentState = ROBOT_STATE_WORKING;
        
        runControl.workMethod  = ROBOT_WORKWAY_HOME;
		
		//DEBUG("isHomeKey\r\n");
    }
	/*Charge����������*/
    else if(runControl.isChargeKey)
    {
		runControl.isChargeKey = false;
		
        if(runControl.lastState == ROBOT_STATE_DEFAULT)       runControl.currentState = ROBOT_STATE_INIT;
        else if(runControl.lastState == ROBOT_STATE_WORKING)  runControl.currentState = ROBOT_STATE_CHARGING;
        else if(runControl.lastState == ROBOT_STATE_CHARGING) runControl.currentState = ROBOT_STATE_SUSPEND;
        else if(runControl.lastState == ROBOT_STATE_SUSPEND)  runControl.currentState = ROBOT_STATE_CHARGING;
        else if(runControl.lastState == ROBOT_STATE_STANDBY)  runControl.currentState = ROBOT_STATE_CHARGING;
        
        runControl.workMethod  = ROBOT_WORKWAY_CHARGE;
		
		//DEBUG("isChargeKey\r\n");
    }
	/*Clean����������*/
    else if(runControl.isCleanKey)
    {
		runControl.isCleanKey = false;
		
        if(runControl.lastState == ROBOT_STATE_DEFAULT)       runControl.currentState = ROBOT_STATE_INIT;
        else if(runControl.lastState == ROBOT_STATE_WORKING)  runControl.currentState = ROBOT_STATE_SUSPEND;
        else if(runControl.lastState == ROBOT_STATE_CHARGING) runControl.currentState = ROBOT_STATE_SUSPEND;
        else if(runControl.lastState == ROBOT_STATE_SUSPEND)  runControl.currentState = ROBOT_STATE_WORKING;
        else if(runControl.lastState == ROBOT_STATE_STANDBY)  runControl.currentState = ROBOT_STATE_WORKING;
        
        runControl.workMethod  = ROBOT_WORKWAY_CLEAN;
		
		//DEBUG("isCleanKey\r\n");
	}
	/*Power����������*/
    else if(runControl.isPowerKey)
    {
		runControl.isPowerKey = false;
		
		DEBUG("�ػ�\r\n");
		
		bsp_InitRunControl();
		bsp_StartRunControl();
		
		bsp_SwOff(SW_5V_EN_CTRL);
		bsp_SwOff(SW_IR_POWER);
    }
	
	switch(runControl.action)
	{
		case 0:/*�жϵ�ǰ״̬�����״̬��ΪROBOT_STATE_DEFAULT��������ִ��*/
		{
			if(runControl.currentState != ROBOT_STATE_DEFAULT)
			{
				runControl.action++;
			}
		}break;
		
		case 1:/*����״̬���򾮲�ͬ�Ĳ���*/
		{
			/*��ʼ��Ӳ��*/
			if(runControl.currentState == ROBOT_STATE_INIT)
			{
				
				/*��ʼ����Ϻ󣬸�����ɨ��ʽ�����в�ͬ�Ĳ���*/
				runControl.currentState = (runControl.workMethod == ROBOT_WORKWAY_CHARGE)? ROBOT_STATE_CHARGING:ROBOT_STATE_WORKING;
				
				DEBUG("��ʼ������...\r\n");
			}
			/*ɨ�ع���*/
			else if(runControl.currentState == ROBOT_STATE_WORKING)
			{
				DEBUG("��ʼ��ɨ...\r\n");
			}
			/*��ͣ*/
			else if(runControl.currentState == ROBOT_STATE_SUSPEND)
			{
				DEBUG("��ͣ...\r\n");
			}
			/*�س�*/
			else if(runControl.currentState == ROBOT_STATE_CHARGING)
			{
				DEBUG("�س�...\r\n");
			}
			/*����*/
			else if(runControl.currentState == ROBOT_STATE_STANDBY)
			{
				DEBUG("����...\r\n");
			}
		}break;
		
	}
}



