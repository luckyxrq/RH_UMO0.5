#include "includes.h"

/*
**********************************************************************************************************
                                            �궨��
**********************************************************************************************************
*/
#define PAUSE_INTERVAL_RESPONSE_TIME         1
#define AT_POWER_ON_OPEN_ALL_MODULE_EN       0     /*�ڿ�����ʱ��ֱ�Ӵ����еĵ������...�����ڵ��Ե�ʱ��ʹ��*/

#define DEBUG_CLOSE_CLEAN_MOTOR              0 //1 �ر���ɨ���

/*
**********************************************************************************************************
                                            ��������
**********************************************************************************************************
*/
/*���� ���������������*/
static void vTaskDecision(void *pvParameters);
/*���� ���ݾ��߿��Ƶ��*/
static void vTaskControl(void *pvParameters);
/*��֪ ��ȡ���������� ����Թܡ����¡���ײ����ء�������������л������������������*/
static void vTaskPerception(void *pvParameters);

static void vTaskMapping(void *pvParameters);

static void AppTaskCreate (void);
static void AppObjCreate (void);
void  App_Printf(char *format, ...);
static void bsp_KeyProc(void);
static void bsp_KeySuspend(void);
static void bsp_UploadBatteryInfo(void);
/*
**********************************************************************************************************
                                            ��������
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskDecision      = NULL;
static TaskHandle_t xHandleTaskControl       = NULL;
static TaskHandle_t xHandleTaskPerception    = NULL;
static TaskHandle_t xHandleTaskMapping       = NULL;


static SemaphoreHandle_t  xMutex = NULL;

bool isSearchCharge = false;
bool isODDStart  = true;

/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: ��׼c������ڡ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int main(void)
{
    /* 
      ����������ǰ��Ϊ�˷�ֹ��ʼ��STM32����ʱ���жϷ������ִ�У������ֹȫ���ж�(����NMI��HardFault)��
      �������ĺô��ǣ�
      1. ��ִֹ�е��жϷ����������FreeRTOS��API������
      2. ��֤ϵͳ�������������ܱ���ж�Ӱ�졣
      3. �����Ƿ�ر�ȫ���жϣ���Ҹ����Լ���ʵ��������ü��ɡ�
      ����ֲ�ļ�port.c�еĺ���prvStartFirstTask�л����¿���ȫ���жϡ�ͨ��ָ��cpsie i������__set_PRIMASK(1)
      ��cpsie i�ǵ�Ч�ġ�
     */
    __set_PRIMASK(1);  
    
    /* Ӳ����ʼ�� */
    bsp_Init(); 
    
    /* 1. ��ʼ��һ����ʱ���жϣ����ȸ��ڵδ�ʱ���жϣ������ſ��Ի��׼ȷ��ϵͳ��Ϣ ��������Ŀ�ģ�ʵ����
          Ŀ�в�Ҫʹ�ã���Ϊ������ܱȽ�Ӱ��ϵͳʵʱ�ԡ�
       2. Ϊ����ȷ��ȡFreeRTOS�ĵ�����Ϣ�����Կ��ǽ�����Ĺر��ж�ָ��__set_PRIMASK(1); ע�͵��� 
    */
    
    
    /* �������� */
    AppTaskCreate();
    
    /* ��������ͨ�Ż��� */
    AppObjCreate();
    
    /* �������ȣ���ʼִ������ */
    vTaskStartScheduler();
    
    /* 
      ���ϵͳ���������ǲ������е�����ģ����е����Ｋ�п��������ڶ�ʱ��������߿��������
      heap�ռ䲻����ɴ���ʧ�ܣ���Ҫ�Ӵ�FreeRTOSConfig.h�ļ��ж����heap��С��
      #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 17 * 1024 ) )
    */
    while(1);
}


/*
*********************************************************************************************************
*	�� �� ��: vTaskStart
*	����˵��: ��������Ҳ����������ȼ�����������������ɨ�衣
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 4  
*********************************************************************************************************
*/
static void vTaskMapping(void *pvParameters)
{
	uint32_t count = 0 ;
	
	
    while(1)
    {
     		
#if 1 /*���µ�ͼ*/
		
		if(isSearchCharge == false)
		{		
			bsp_GridMapUpdate(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(),bsp_CollisionScan(),bsp_GetIRSensorData(),bsp_GetCliffSensorData());
		}
#endif
		
		//bsp_UploadMap();
		if(count++ % 100 == 0)
		{	
			bsp_UploadBatteryInfo();
		}
		
		
		//bsp_PrintCollision();
		
        vTaskDelay(100);
    }

}


/*
*********************************************************************************************************
*	�� �� ��: vTaskDecision
*	����˵��: ���� ���������������
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 4  
*********************************************************************************************************
*/
static void vTaskDecision(void *pvParameters)
{
    
    uint32_t count = 0 ;
	vTaskDelay(2000);
    while(1)
    {
        /* �������¼� */
        bsp_KeyProc();
		
        if(count++ % 10 == 0)
        {
			bsp_ChangeWifi2SmartConfigStateProc();
			
			/*�����Ǵ�ӡ���أ�����ע��*/
			bsp_WifiStateProc();
        }
		
        vTaskDelay(50);	
    }

}

/*
*********************************************************************************************************
*	�� �� ��: vTaskStart
*	����˵��: ��������Ҳ����������ȼ�����������������ɨ�衣
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 4  
*********************************************************************************************************
*/
static void vTaskControl(void *pvParameters)       //���� ���ݾ��߿��Ƶ��
{
	uint32_t count = 0 ;
	
    while(1)
    {
#if 0
        bsp_IWDG_Feed(); /* ι�� */
#endif
        		
#if 0		
        DEBUG("L %d MM/S\r\n",bsp_MotorGetSpeed(MotorLeft));
        DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
#endif		
#if 0
		if(count%100 ==0)
		{
			bsp_CleanZeroYaw();
		}
#endif
		
		if(isSearchCharge == false)
		{	
			bsp_UpdateCleanStrategyB(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(), bsp_CollisionScan(), \
			bsp_MotorGetPulseVector(MotorLeft), bsp_MotorGetPulseVector(MotorRight), bsp_GetIRSensorData(),bsp_GetCliffSensorData());
			
		}
		if(GetReturnChargeStationStatus())
		{
			bsp_StopUpdateCleanStrategyB();
			ResetReturnChargeStationStatus();
			bsp_PutKey(KEY_LONG_CHARGE);
		}
		
		bsp_PumpProc();
		
		count++;
        vTaskDelay(20);
    }
    
}


bool isNeedRun = false;

/*
*********************************************************************************************************
*	�� �� ��: vTaskStart
*	����˵��: ��֪ ��ȡ���������� ����Թܡ����¡���ײ����ء�������������л�����������������ǡ�
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 4  
*********************************************************************************************************
*/
static void vTaskPerception(void *pvParameters)
{
	uint32_t count = 0 ;
	
    /*��������Թ���ѯɨ��*/
    bsp_DetectStart(); 
	
	/*�����������*/
	bsp_StartOffSiteProc();
	
	/*�򿪼�⳾��*/
	bsp_StartDustBoxProc();
	
	/*����λ���������*/
    bsp_StartUpdatePos();
	
	/*����դ���ͼ����*/
	bsp_StartUpdateGridMap();

	/*��������ģʽ���*/
	bsp_StartSleepProc();
	
	/*��ʼ�����´�����*/
	bsp_InitCliffSW();
	
	
	
#if AT_POWER_ON_OPEN_ALL_MODULE_EN /*�ڿ�����ʱ��ֱ�Ӵ����еĵ������...�����ڵ��Ե�ʱ��ʹ��*/
	bsp_StartVacuum(VACUUM_DEFAULT_PER);
	bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
#endif
	
    while(1)
    {
		bsp_ComAnalysis();
		
#if 0
		if(bsp_IsInitAW9523B_OK())
		{
			bsp_DetectAct();  /*����Թ���ѯɨ��*/
			bsp_DetectDeal(); /*����Թ�ɨ��������*/
		}
#endif
       
		/*�������*/
		bsp_StrategyRandomProc();
		
		/*���Դ�����*/
		bsp_FunctionTestUpdate();
		
		/*�����������*/
		if(!GetCmdStartUpload())
		{
			bsp_OffSiteProc();
		}
		
		/*��⳾��*/
//		if(!GetCmdStartUpload())
//		{
//			bsp_DustBoxProc();
//		}
		
        /*Ѱ�ҳ��׮*/
		bsp_SearchChargePile();
		/*�ر�����*/
		bsp_EdgewiseRun();
        /*��������*/
        bsp_PositionUpdate();
		bsp_LedAppProc();
		wifi_uart_service();
		
		/*�������´�������Ϣ*/
		bsp_GetCliffStates();
		
		/*�Լ����*/
		bsp_SelfCheckProc();
		
		/*��������ģʽ���*/
		bsp_SleepProc();
		
		/*�ϴ����غ�ʱ����ͬʱ����*/
		if(GetCmdStartUpload() && count % 50 == 0)
		{
			bsp_SendReportFrameWithCRC16();
		}
		
		count++;
        vTaskDelay(5);	
    }		
    
}


/*
*********************************************************************************************************
*	�� �� ��: AppTaskCreate
*	����˵��: ����Ӧ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
	
	xTaskCreate( vTaskMapping,     		        /* ������  */
                 "vTaskMapping",   		        /* ������    */
                 1024*2,            		    /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 1,              		        /* �������ȼ�*/
                 &xHandleTaskMapping );         /* ������  */
    xTaskCreate( vTaskDecision,     		    /* ������  */
                 "vTaskDecision",   		    /* ������    */
                 512,            		        /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 2,              		        /* �������ȼ�*/
                 &xHandleTaskDecision );        /* ������  */
    xTaskCreate( vTaskControl,     		        /* ������  */
                 "vTaskControl",   		        /* ������    */
                 1024,            		        /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 3,              		        /* �������ȼ�*/
                 &xHandleTaskControl );         /* ������  */	
    xTaskCreate( vTaskPerception,     		    /* ������  */
                 "vTaskPerception",   		    /* ������    */
                 1024,            		        /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 4,              		        /* �������ȼ�*/
                 &xHandleTaskPerception );      /* ������  */	
	
}

/*
*********************************************************************************************************
*	�� �� ��: AppObjCreate
*	����˵��: ��������ͨ�Ż���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AppObjCreate (void)
{
    /* ���������ź��� */
    xMutex = xSemaphoreCreateMutex();
    
    if(xMutex == NULL)
    {
        /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
    }
}

/*
*********************************************************************************************************
*	�� �� ��: App_Printf
*	����˵��: �̰߳�ȫ��printf��ʽ		  			  
*	��    ��: ͬprintf�Ĳ�����
*             ��C�У����޷��г����ݺ���������ʵ�ε����ͺ���Ŀʱ,������ʡ�Ժ�ָ��������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void  App_Printf(char *format, ...)
{
    char  buf_str[200 + 1];
    va_list   v_args;
    
    
    va_start(v_args, format);
    (void)vsnprintf((char       *)&buf_str[0],
            (size_t      ) sizeof(buf_str),
            (char const *) format,
            v_args);
    va_end(v_args);
    
    /* �����ź��� */
    xSemaphoreTake(xMutex, portMAX_DELAY);
    
    DEBUG("%s", buf_str);
    
    xSemaphoreGive(xMutex);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_UploadBatteryInfo
*	����˵��: ��		  			  
*	��    ��: �ϴ������Ϣ��APP
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_UploadBatteryInfo(void)
{
	float battery_adc_value = 0;
	uint32_t battery_precent = 0 ;

	battery_adc_value  = bsp_GetFeedbackVoltage(eBatteryVoltage);
	battery_adc_value  = ((battery_adc_value * 430.0f / 66.5f) + battery_adc_value + 0.2F);
	battery_precent = ((battery_adc_value - 11.9f) / 4.8f)*100; 
	mcu_dp_value_update(DPID_RESIDUAL_ELECTRICITY, battery_precent);
	mcu_dp_value_update(DPID_CLEAN_TIME, RealWorkTime/1000/60);
}

static void bsp_StopAllMotor(void)
{
	bsp_SetMotorSpeed(MotorLeft, 0);
	bsp_SetMotorSpeed(MotorRight,0);
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
	bsp_MotorCleanSetPWM(MotorRollingBrush, CW , 0);
	bsp_StopVacuum();
}


typedef enum
{
	eKEY_NONE = 0,
	eKEY_CLEAN
}KEY_STATE;

static KEY_STATE key_state = eKEY_NONE;

void bsp_SetLastKeyState(KEY_STATE state)
{
	key_state = state;
}

KEY_STATE bsp_GetLastKeyState(void)
{
	return key_state;
}


void bsp_CloseAllStateRun(void)
{
	bsp_StopSearchChargePile();
	bsp_StopCliffTest();
	bsp_StopUpdateCleanStrategyB();
	bsp_StopStrategyRandom();
	bsp_StopEdgewiseRun();
}

void bsp_OffsiteSuspend(void)
{
	/*�ƹ���3�Ű�ɫ��*/
	bsp_OpenThreeWhileLed();
	bsp_SetLedState(LED_DEFAULT_STATE);
	
	/*�ر�����״̬��*/
	bsp_CloseAllStateRun();
	
	/*�ر����е��*/
	bsp_StopAllMotor();

	
	/*������һ�ΰ���ֵ*/
	bsp_SetLastKeyState(eKEY_NONE);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_KeySuspend
*	����˵��: ��ͬ״̬��ͣ����Ч����ͬ	  			  
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_KeySuspend(void)
{
	/*�ƹ���3�Ű�ɫ��*/
	bsp_OpenThreeWhileLed();
	bsp_SetLedState(LED_DEFAULT_STATE);
	
	/*�ر����е��*/
	bsp_StopAllMotor();
	
	/*�ر�ˮ��*/
	bsp_StopPump();
	
	/*�ر�����״̬��*/
	bsp_CloseAllStateRun();

	/*��һ������ɨ�����ξͲ�����ͣ��ɨ*/
	if(bsp_GetLastKeyState() == eKEY_CLEAN)
	{
		bsp_SperkerPlay(Song4);
	}
	
	/*������һ�ΰ���ֵ*/
	bsp_SetLastKeyState(eKEY_NONE);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_KeyProc
*	����˵��: ����������	  			  
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_KeyProc(void)
{
	uint8_t ucKeyCode;	
	
	ucKeyCode = bsp_GetKey();
	if (ucKeyCode > 0)
	{
		/* �м����� */
		switch (ucKeyCode)
		{
			case KEY_DOWN_POWER:
			{
				DEBUG("KEY_DOWN_POWER\r\n");
				bsp_KeySuspend();
				isNeedRun = true;
			}break;
				
			case KEY_DOWN_CHARGE:
			{
				DEBUG("KEY_DOWN_CHARGE\r\n");
				bsp_KeySuspend();
			}break;
				
			case KEY_DOWN_CLEAN:	
			{
				DEBUG("KEY_DOWN_CLEAN\r\n");
				bsp_KeySuspend();
			}break;
			

			
			case KEY_LONG_POWER: /*�ػ�*/
			{
				DEBUG("��Դ��������\r\n");
				
				/*�ƹ���3�Ű�ɫ��*/
				bsp_CloseAllLed();
				bsp_SetLedState(LED_DEFAULT_STATE);
				
				/*�ر����е��*/
				bsp_StopAllMotor();
				
				/*�ر�����״̬��*/
				bsp_CloseAllStateRun();

				/*������һ�ΰ���ֵ*/
				bsp_SetLastKeyState(eKEY_NONE);
				/*��������ģʽ*/
				bsp_SperkerPlay(Song31);
				vTaskDelay(10);	
				while(bsp_SpeakerIsBusy()){}
				
				bsp_ClearKey();
				bsp_EnterStopMODE();
			}break;
			
			case KEY_LONG_CHARGE: /*���*/	
			{
				DEBUG("��簴������\r\n");

				/*�����ж��Ƿ���������*/
				if(!GetCmdStartUpload() && bsp_OffSiteGetState() != OffSiteNone) /*ǰ�᲻�����ϴ�״̬*/
				{
					bsp_SperkerPlay(Song16);
					return;
				}
				
				/*�����жϳ���*/
				if(!GetCmdStartUpload() && bsp_DustBoxGetState() == DustBoxOutside) /*ǰ�᲻�����ϴ�״̬*/
				{
					bsp_SperkerPlay(Song9);
					return;
				}
				
				bsp_SperkerPlay(Song5);
				bsp_StartSearchChargePile();
				bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.6F);
				/*������һ�ΰ���ֵ*/
				bsp_SetLastKeyState(eKEY_NONE);
				/*����LED״̬*/
				bsp_SetLedState(AT_SEARCH_CHARGE);
				isSearchCharge = true;
				bsp_ClearKey();
			}break;
			
			case KEY_LONG_CLEAN: /*��ɨ*/
			{
				DEBUG("��ɨ��������\r\n");
				
				/*�����ж��Ƿ���������*/
				if(!GetCmdStartUpload() && bsp_OffSiteGetState() == OffSiteBoth)   /*ǰ�᲻�����ϴ�״̬*/
				{
					bsp_SperkerPlay(Song16);
					return;
				}

				bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.8F);
				bsp_StartPump();
				bsp_StartStrategyRandom();
				
				
				/*������һ�ΰ���ֵ*/
				bsp_SetLastKeyState(eKEY_CLEAN);
				/*����LED״̬*/
				bsp_SetLedState(AT_CLEAN);
				isSearchCharge = false;
				bsp_ClearKey();
				
			}break;
			
			case KEY_9_DOWN:
			{
				DEBUG("����������ͬʱ��������ɨ\r\n");
				bsp_SperkerPlay(Song29);
				bsp_StartChangeWifi2SmartConfigState();
				bsp_SetLedState(AT_LINK);
			}break;
			
			
			case KEY_10_LONG:
			{
				bsp_StartFunctionTest();
			}break;
			
			
			case KEY_WIFI_OPEN_CLEAN_CAR:
			{
				bsp_SperkerPlay(Song1);
			}break;
			
			case KEY_WIFI_CLOSE_CLEAN_CAR:
			{
				bsp_SperkerPlay(Song2);
				
				/*�ƹ���3�Ű�ɫ��*/
				bsp_OpenThreeWhileLed();
				bsp_SetLedState(LED_DEFAULT_STATE);
				
				/*�ر����е��*/
				bsp_StopAllMotor();
				
				/*�ر�����״̬��*/
				bsp_CloseAllStateRun();
				
				/*������һ�ΰ���ֵ*/
				bsp_SetLastKeyState(eKEY_NONE);
			}break;
			
			case KEY_WIFI_DIR_FRONT:
			{
				bsp_KeySuspend();
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(250));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
				vTaskDelay(1000);	
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			}break;
			
			case KEY_WIFI_DIR_BACK:
			{
				bsp_KeySuspend();
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-250));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-250));
				vTaskDelay(1000);	
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			}break;
			
			case KEY_WIFI_DIR_LEFT:
			{
				bsp_KeySuspend();
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-150));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+150));
				vTaskDelay(1500);	
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			}break;
			
			case KEY_WIFI_DIR_RIGHT:
			{
				bsp_KeySuspend();
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+150));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-150));
				vTaskDelay(1500);	
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			}break;
			
			case KEY_WIFI_EDGE:
			{
				bsp_KeySuspend();
				bsp_SperkerPlay(Song34);
				bsp_StartEdgewiseRun();
			}break;
		}   
	}
}





/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
