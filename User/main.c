#include "includes.h"

/*
**********************************************************************************************************
                                            �궨��
**********************************************************************************************************
*/
#define PAUSE_INTERVAL_RESPONSE_TIME         1
#define AT_POWER_ON_OPEN_ALL_MODULE_EN       0     /*�ڿ�����ʱ��ֱ�Ӵ����еĵ������...�����ڵ��Ե�ʱ��ʹ��*/

#define DEBUG_CLOSE_CLEAN_MOTOR              0 //1 �ر���ɨ���
#define main_debug(format, ...) //printf (format, ##__VA_ARGS__)
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
	uint32_t battery = 0 ;
    while(1)
    {
     		
#if 1 /*���µ�ͼ*/
		
		if(isSearchCharge){}
		else{		
			main_debug("bsp_GridMapUpdate() \n");
			bsp_GridMapUpdate(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(),bsp_CollisionScan(),bsp_GetIRSensorData(),bsp_GetCliffSensorData());

		}
#endif
		
		//bsp_UploadMap();
		if(count % 20 == 0)
		{
			mcu_dp_value_update(DPID_RESIDUAL_ELECTRICITY,++battery % 100);
		}
        vTaskDelay(100);
		
		
    }

}


static void vTaskDecision(void *pvParameters)      //���� ���������������
{
    
    uint32_t count = 0 ;

    while(1)
    {
        /* �������¼� */
		main_debug("bsp_KeyProc() \n");
        bsp_KeyProc();
		
		
        if(count++ % 10 == 0)
        {
#if 0 
			bsp_PrintIR_Rev(); /*���ڴ�ӡ�������״̬*/
#endif
			//main_debug("bsp_ChangeWifi2SmartConfigStateProc() \n");
			bsp_ChangeWifi2SmartConfigStateProc();
			
			/*�����Ǵ�ӡ���أ�����ע��*/
			//main_debug("bsp_WifiStateProc() \n");
			bsp_WifiStateProc();
//			bsp_PrintCollision();
//			bsp_PrintIR_Rev();
//			bsp_PrintAllVoltage();
			/*�������´�������Ϣ*/
			bsp_GetCliffStates();
//			bsp_PrintCliff();
			bsp_SendReportFrameWithCRC16();
			
			//DEBUG("bsp_AngleReadRaw:%d\r\n",bsp_AngleReadRaw());
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
			DEBUG("bsp_CleanZeroYaw()\r\n");
		}
		
		DEBUG("Orientation: %f �� \r\n", Rad2Deg(bsp_GetCurrentOrientation()));
#endif
		
		
		if(isSearchCharge)
		{
		
		}
		else
		{	
			main_debug("bsp_UpdateCleanStrategyB() \n");
			bsp_UpdateCleanStrategyB(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(), bsp_CollisionScan(), \
			bsp_MotorGetPulseVector(MotorLeft), bsp_MotorGetPulseVector(MotorRight), bsp_GetIRSensorData(),bsp_GetCliffSensorData());
			
		}//DEBUG("%+4d,%+4d#%+3d \n",bsp_GetCurrentPosX()/10,bsp_GetCurrentPosY()/10,(int)Rad2Deg(bsp_GetCurrentOrientation()));
		
		
		
		if(GetReturnChargeStationStatus())
		{
			
			//main_debug("bsp_StopUpdateCleanStrategyB() \n");
			bsp_StopUpdateCleanStrategyB();
			
			
			//main_debug("ResetReturnChargeStationStatus() \n");
			ResetReturnChargeStationStatus();
			
			
			//main_debug("bsp_PutKey(KEY_LONG_CHARGE) \n");
			bsp_PutKey(KEY_LONG_CHARGE);
		}
		
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
float roll = 0.0F;
static void vTaskPerception(void *pvParameters)
{
	uint32_t count = 0 ;
	
    /*��������Թ���ѯɨ��*/
    bsp_DetectStart(); 
	
	/*�����������*/
	bsp_StartOffSiteProc();
	
	bsp_StartDustBoxProc();
	
	/*����Ѱ�ҳ��׮*/
	//bsp_StartSearchChargePile();
	
	/*�����ر�����*/
	//bsp_StartEdgewiseRun();
	
	/*����λ���������*/
    bsp_StartUpdatePos();
	
    /*����������ײЭ��*/
	//bsp_StartAssistJudgeDirection();
	
	/*����դ���ͼ����*/
	bsp_StartUpdateGridMap();

	/*����ɨ����*/
	//bsp_StartUpdateCleanStrategyB();
	
	//bsp_StartCliffTest();

	vTaskDelay(2000);		
	
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
		bsp_ComAnalysis();
		//bsp_CliffTest();
		
		//roll = bsp_IMU_GetData(ROLL)*100*0.01F;
		//DEBUG("%.2F\r\n",roll);
//		if(roll <= 175 && roll >= 0)
//		{
//			bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(500));
//			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(500));
//		}
//		else
//		{
//			bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
//			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
//		}
		
		
//		if(isNeedRun)
//		{
//			if(bsp_CliffIsDangerous(CliffLeft) || bsp_CliffIsDangerous(CliffMiddle) || bsp_CliffIsDangerous(CliffRight))
//			{
//				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
//				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
//				isNeedRun = false;
//			}
//			else
//			{
//				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
//				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
//			}
//		}
		
#if 1
		if(bsp_IsInitAW9523B_OK())
		{
			//main_debug("bsp_DetectAct() \n");
			bsp_DetectAct();  /*����Թ���ѯɨ��*/
			//main_debug("bsp_DetectDeal() \n");
			bsp_DetectDeal(); /*����Թ�ɨ��������*/
		}
#endif
       
#if 0   /*���Ժ�����ľ��룬�⵽���ͣ����*/
		bsp_DetectMeasureTest();
#endif

#if 0  /*�������´����� �����⡢��ײ��ͬ����*/	 
		//main_debug("bsp_CliffTest() \n");
		bsp_CliffTest();
#endif
		
#if 0  /*����IMU�����Ƿ�����*/	 
		DEBUG("bsp_AngleReadRaw:%d\n",bsp_AngleReadRaw());
#endif
		
#if 0  /*���Ե�ص�ѹ�����Ƿ�����*/	 
		
		DEBUG("BatteryVoltage:%f\n",((bsp_GetFeedbackVoltage(eBatteryVoltage) * 430 / 66.5) + bsp_GetFeedbackVoltage(eBatteryVoltage) + 0.2F));
#endif
		
		
		

		bsp_StrategyRandomProc();
		
		/*�����������*/
		//main_debug("bsp_OffSiteProc() \n");
		//bsp_OffSiteProc();
		//check dust box
		bsp_DustBoxProc();
        /*Ѱ�ҳ��׮*/
		//main_debug("bsp_SearchChargePile() \n");
		bsp_SearchChargePile();
		/*�ر�����*/
		//main_debug("bsp_EdgewiseRun() \n");
		bsp_EdgewiseRun();
        /*��������*/
		//main_debug("bsp_PositionUpdate() \n");
        bsp_PositionUpdate();
		//main_debug("bsp_LedAppProc() \n");
		bsp_LedAppProc();
		
		//main_debug("wifi_uart_service() \n");
		wifi_uart_service();
		
		

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
	
	xTaskCreate( vTaskMapping,     		    /* ������  */
                 "vTaskMapping",   		    /* ������    */
                 1024*1.5,            		    /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 1,              		        /* �������ȼ�*/
                 &xHandleTaskMapping );        /* ������  */
    xTaskCreate( vTaskDecision,     		    /* ������  */
                 "vTaskDecision",   		    /* ������    */
                 512,            		    /* ����ջ��С����λword��Ҳ����4�ֽ� */
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
    
    printf("%s", buf_str);
    
    xSemaphoreGive(xMutex);
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
				DEBUG("��Դ��������\r\n");
				bsp_KeySuspend();
				isNeedRun = true;
			}break;
				
			case KEY_DOWN_CHARGE:
			{
				DEBUG("��簴������\r\n");
				bsp_KeySuspend();
			}break;
				
			case KEY_DOWN_CLEAN:	
			{
				DEBUG("��ɨ��������\r\n");
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
//				if(bsp_OffSiteGetState() != OffSiteNone)
//				{
//					bsp_SperkerPlay(Song16);
//					return;
//				}
				
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
//				if(bsp_OffSiteGetState() == OffSiteBoth)
//				{
//					bsp_SperkerPlay(Song16);
//					return;
//				}
				
				/*�����жϳ���*/
				if(bsp_DustBoxGetState() == DustBoxOutside)
				{
					bsp_SperkerPlay(Song9);
					return;
				}
				
				bsp_SperkerPlay(Song3);
				
				
				/*�������*/
			    if(bsp_IsTouchChargePile())
				{
					bsp_CleanZeroYaw();
					vTaskDelay(1000);
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-150));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-150));
					vTaskDelay(1000);
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					vTaskDelay(500);
					if(isODDStart) 
					{	
						bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+200));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-200));
					}
					else
					{
						bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-200));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+200));
					}
					vTaskDelay(1900);
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					vTaskDelay(500);
					if(isODDStart) 
					{	
						isODDStart = false;
					}
					else
					{
						isODDStart = true;
						bsp_CleanZeroYaw();
					}
						
					vTaskDelay(50);
				}
					
				bsp_StartUpdateCleanStrategyB();
				//bsp_StartEdgewiseRun();
				
				if(!DEBUG_CLOSE_CLEAN_MOTOR){
				bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				bsp_StartVacuum();
				}
				
				
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
			
			
			case KEY_10_DOWN:
			{
				
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
