#include "includes.h"

/*
**********************************************************************************************************
                                            �궨��
**********************************************************************************************************
*/
#define PAUSE_INTERVAL_RESPONSE_TIME         400
#define AT_POWER_ON_OPEN_ALL_MODULE_EN       0     /*�ڿ�����ʱ��ֱ�Ӵ����еĵ������...�����ڵ��Ե�ʱ��ʹ��*/

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

static void AppTaskCreate (void);
static void AppObjCreate (void);
void  App_Printf(char *format, ...);
static void bsp_KeySuspend(void);
static void bsp_KeyProc(void);
/*
**********************************************************************************************************
                                            ��������
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskDecision      = NULL;
static TaskHandle_t xHandleTaskControl       = NULL;
static TaskHandle_t xHandleTaskPerception    = NULL;

static SemaphoreHandle_t  xMutex = NULL;
static KeyProc keyProc;
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
    vSetupSysInfoTest();
    
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
static void vTaskDecision(void *pvParameters)      //���� ���������������
{
    
    uint32_t count = 0 ;
    
	
    bsp_AngleRst();
	bsp_SperkerPlay(Song1);
	
    while(1)
    {
        /* �������¼� */
        bsp_KeyProc();
		
		
        if(count++ % 10 == 0)
        {
#if 0 
			bsp_PrintIR_Rev(); /*���ڴ�ӡ�������״̬*/
#endif
			bsp_ChangeWifi2SmartConfigStateProc();
			
			/*�����Ǵ�ӡ���أ�����ע��*/
			bsp_WifiStateProc();
			//bsp_PrintCollision();
			//bsp_PrintIR_Rev();
        }
		
#if 1 /*���µ�ͼ*/
		
		//DEBUG("Start:%d\r\n",xTaskGetTickCount());
		bsp_GridMapUpdate(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(),bsp_CollisionScan(),bsp_GetIRSensorData(),bsp_GetCliffSensorData());
		//DEBUG("X:%d,Y:%d#\n",bsp_GetCurrentPosX(),bsp_GetCurrentPosY());
		//DEBUG("End:%d\r\n",xTaskGetTickCount());
#endif

		
		//bsp_UploadMap();
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
		
		
        bsp_ComAnalysis();
		bsp_PowerOnToggle();/* ����״̬�� */ 
		bsp_RunToggleLED();
		
		if(count %2 ==0)
		{
			bsp_CleanStrategyUpdateB(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(), bsp_CollisionScan(), \
			bsp_MotorGetPulseVector(MotorLeft), bsp_MotorGetPulseVector(MotorRight), bsp_GetIRSensorData(),bsp_GetCliffSensorData());
			//DEBUG("%+4d,%+4d#%+3d \n",bsp_GetCurrentPosX()/10,bsp_GetCurrentPosY()/10,(int)Rad2Deg(bsp_GetCurrentOrientation()));
		}
		
		
		count++;
        vTaskDelay(10);
    }
    
}


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
	//bsp_StartOffSiteProc();
	
	/*����Ѱ�ҳ��׮*/
	//bsp_StartSearchChargePile();
	
	/*�����ر�����*/
	//bsp_StartEdgewiseRun();
	
	/*����λ���������*/
    bsp_StartUpdatePos();
	
    /*����������ײЭ��*/
	//bsp_StartAssistJudgeDirection();
	
	/*����դ���ͼ����*/
	//bsp_StartUpdateGridMap();

	/*����ɨ����*/
	//bsp_StartUpdateCleanStrategyB();
	
	//bsp_StartCliffTest();

	

	if( !bsp_IsSelfCheckingReady())
	{
		 vTaskDelay(1);	
	}	
	
	
	bsp_InitCliffSW();
	
#if AT_POWER_ON_OPEN_ALL_MODULE_EN /*�ڿ�����ʱ��ֱ�Ӵ����еĵ������...�����ڵ��Ե�ʱ��ʹ��*/
	bsp_StartVacuum();
	bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
#endif
	
	
    while(1)
    {
#if 1
        bsp_DetectAct();  /*����Թ���ѯɨ��*/
        bsp_DetectDeal(); /*����Թ�ɨ��������*/
#endif
		
       
#if 0   /*���Ժ�����ľ��룬�⵽���ͣ����*/
		bsp_DetectMeasureTest();
#endif

#if 0   /*�������´����� �����⡢��ײ��ͬ����*/	 	
		bsp_CliffTest();
#endif
		
	
		/*�����������*/
		bsp_OffSiteProc();
        /*Ѱ�ҳ��׮*/
		bsp_SearchChargePile();
		/*�ر�����*/
		bsp_EdgewiseRun();
        /*��������*/
        bsp_PositionUpdate();
		

		if(count % 10 == 0)
		{
			bsp_PidSched(); /*10MS����һ�Σ����������PWM���㣬ռ�ձ����ã��ٶȣ�����Ϊ��λ��MMΪ��λ������*/
			//bsp_AssistJudgeDirection();
		}
		
		
		
		wifi_uart_service();
		
		count++;
        vTaskDelay(1);	
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
    xTaskCreate( vTaskDecision,     		    /* ������  */
                 "vTaskDecision",   		    /* ������    */
                 1024,            		        /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 1,              		        /* �������ȼ�*/
                 &xHandleTaskDecision );        /* ������  */
    xTaskCreate( vTaskControl,     		        /* ������  */
                 "vTaskControl",   		        /* ������    */
                 1024,            		        /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 2,              		        /* �������ȼ�*/
                 &xHandleTaskControl );         /* ������  */	
    xTaskCreate( vTaskPerception,     		    /* ������  */
                 "vTaskPerception",   		    /* ������    */
                 1024,            		        /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 3,              		        /* �������ȼ�*/
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
    
    printf("%s", buf_str);
    
    xSemaphoreGive(xMutex);
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
	RunState lastRunState = bsp_GetKeyRunLastState();
	
	if(lastRunState != RUN_STATE_DEFAULT)
	{
		/*��¼�¶̰���ʱ��*/
		bsp_SetLastKeyTick(xTaskGetTickCount());
		
		if(lastRunState == RUN_STATE_CHARGE)
		{
			bsp_StopRunToggleLED();
			bsp_StopVacuum();
		}
		else if(lastRunState == RUN_STATE_CLEAN)
		{
			bsp_SperkerPlay(Song4);
			bsp_StopRunToggleLED();
			bsp_StopVacuum();
		}
		else if(lastRunState == RUN_STATE_SHUTDOWN)
		{
			bsp_LedOn(LED_LOGO_CLEAN);
			bsp_LedOn(LED_LOGO_POWER);
			bsp_LedOn(LED_LOGO_CHARGE);
			bsp_LedOff(LED_COLOR_YELLOW);
			bsp_LedOff(LED_COLOR_GREEN);
			bsp_LedOff(LED_COLOR_RED);
		}
		
		bsp_StopSearchChargePile();
		bsp_StopCliffTest();
		bsp_SetKeyRunLastState(RUN_STATE_DEFAULT);
		
		bsp_StopUpdateCleanStrategyB();
		bsp_MotorCleanSetPWM(MotorRollingBrush, CW , 0);
		bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
	}
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_SetKeyRunLastState
*	����˵��: �����ϴεİ���״̬	  			  
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetKeyRunLastState(RunState state)
{
	keyProc.lastRunState = state;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetKeyRunLastState
*	����˵��: ��ȡ�ϴεİ���״̬	  			  
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
RunState bsp_GetKeyRunLastState(void)
{
	return keyProc.lastRunState;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetLastKeyTick
*	����˵��: ��¼�ϴΰ�����ʱ�� 			  
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetLastKeyTick(uint32_t tick)
{
	keyProc.lastKeyTick = tick;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetLastKeyTick
*	����˵��: ��ȡ�ϴΰ�����ʱ�� 			  
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint32_t bsp_GetLastKeyTick(void)
{
	return keyProc.lastKeyTick;
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
//	uint8_t ucKeyCode;	
//	
//	ucKeyCode = bsp_GetKey();
//	//if (ucKeyCode > 0 && bsp_IsSelfCheckingReady())
//	if (ucKeyCode > 0)
//	{
//		/* �м����� */
//		switch (ucKeyCode)
//		{
//			case KEY_DOWN_POWER:
//			{
//				DEBUG("��Դ��������\r\n");
//				bsp_KeySuspend();
//			}break;
//				
//			case KEY_DOWN_CHARGE:
//			{
//				DEBUG("��簴������\r\n");
//				bsp_KeySuspend();
//			}break;
//				
//			case KEY_DOWN_CLEAN:	
//			{
//				DEBUG("��ɨ��������\r\n");
//				bsp_KeySuspend();
//			}break;
//			

//			
//			case KEY_LONG_POWER: /*�ػ�*/
//			{
//				DEBUG("��Դ��������\r\n");
//				if(xTaskGetTickCount() - bsp_GetLastKeyTick() >= PAUSE_INTERVAL_RESPONSE_TIME)
//				{
//					bsp_SetKeyRunLastState(RUN_STATE_SHUTDOWN);
//					bsp_SperkerPlay(Song31);
//					
//					bsp_LedOff(LED_LOGO_CLEAN);
//					bsp_LedOff(LED_LOGO_POWER);
//					bsp_LedOff(LED_LOGO_CHARGE);
//					bsp_LedOff(LED_COLOR_YELLOW);
//					bsp_LedOff(LED_COLOR_GREEN);
//					bsp_LedOff(LED_COLOR_RED);
//					
//					vTaskDelay(100);	
//					while(bsp_SpeakerIsBusy()){}
//					bsp_ClearKey();
//						
//					bsp_EnterStopMODE();
//				}
//				
//			}break;
//			
//			case KEY_LONG_CHARGE: /*���*/	
//			{
//				DEBUG("��簴������\r\n");
//				if(xTaskGetTickCount() - bsp_GetLastKeyTick() >= PAUSE_INTERVAL_RESPONSE_TIME)
//				{
//					bsp_SetKeyRunLastState(RUN_STATE_CHARGE);
//					bsp_SperkerPlay(Song5);
//					bsp_StartRunToggleLED(LED_LOGO_CHARGE);
//					//bsp_StartCliffTest();
//					bsp_StartSearchChargePile();
//					
//					vTaskDelay(200);	
//					while(bsp_SpeakerIsBusy()){}
//					bsp_ClearKey();
//				}
//				
//			}break;
//			
//			case KEY_LONG_CLEAN: /*��ɨ*/
//			{
//				DEBUG("��ɨ��������\r\n");
//				if(xTaskGetTickCount() - bsp_GetLastKeyTick() >= PAUSE_INTERVAL_RESPONSE_TIME)
//				{
//					bsp_SetKeyRunLastState(RUN_STATE_CLEAN);
//					bsp_SperkerPlay(Song3);
//					bsp_StartRunToggleLED(LED_LOGO_CLEAN);
//					
//					//bsp_StartCliffTest();
//					/*����ɨ����*/
//					bsp_StartUpdateCleanStrategyB();
//					bsp_StartVacuum();
//					bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
//					bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
//					
//					//bsp_StartEdgewiseRun();
//					
//					vTaskDelay(200);	
//					while(bsp_SpeakerIsBusy()){}
//					bsp_ClearKey();
//				}
//				
//			}break;
//			
//			case KEY_9_DOWN:
//			{
//				bsp_StopRunToggleLED();
//				
//				/*��λ��һ�εİ���״̬*/
//				bsp_SetKeyRunLastState(RUN_STATE_DEFAULT);
//				
//				
//				
//				/*�رո���״̬��*/
//				bsp_StopCliffTest();
//				bsp_StopVacuum();
//				/*�رյ��*/
//				bsp_SetMotorSpeed(MotorLeft, 0);
//				bsp_SetMotorSpeed(MotorRight,0);
//				bsp_StartEdgewiseRun();
//				
//			}break;
//			
//			
//			case KEY_10_DOWN:
//			{
//				DEBUG("����������ͬʱ��������ɨ\r\n");
//				bsp_SperkerPlay(Song29);
//				bsp_StartChangeWifi2SmartConfigState();
//				bsp_StartRunToggleLED(LED_WIFI_LINK);
//			}break;
//		}   
//	}
}



/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
