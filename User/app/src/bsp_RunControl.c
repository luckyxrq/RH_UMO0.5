#include "bsp.h"


static RunControl runControl;


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
		
		DEBUG("isHomeKey\r\n");
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
		
		DEBUG("isChargeKey\r\n");
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
		
		DEBUG("isCleanKey\r\n");
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
			}
			/*ɨ�ع���*/
			else if(runControl.currentState == ROBOT_STATE_WORKING)
			{
				
			}
			/*��ͣ*/
			else if(runControl.currentState == ROBOT_STATE_SUSPEND)
			{
				
			}
			/*�س�*/
			else if(runControl.currentState == ROBOT_STATE_CHARGING)
			{
				
			}
			/*����*/
			else if(runControl.currentState == ROBOT_STATE_STANDBY)
			{
				
			}
		}break;
		
		
		
		
	}
}
