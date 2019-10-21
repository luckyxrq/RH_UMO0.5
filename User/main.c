#include "includes.h"

/*
**********************************************************************************************************
                                            �궨��
**********************************************************************************************************
*/
#define PAUSE_INTERVAL_RESPONSE_TIME      400

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
	bsp_SetMotorSpeed(MotorLeft ,  12*1.5);
	bsp_SetMotorSpeed(MotorRight , 12*1.5);
	
    while(1)
    {
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

	
    while(1)
    {

        		
#if 1		
        DEBUG("L %d MM/S  ",bsp_MotorGetSpeed(MotorLeft));
        DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
#endif		
		
		bsp_PidSched(); /*10MS����һ�Σ����������PWM���㣬ռ�ձ����ã��ٶȣ�����Ϊ��λ��MMΪ��λ������*/
		
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

	
    while(1)
    {
		bsp_KeyScan();

		
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
	uint8_t ucKeyCode;	
	
	ucKeyCode = bsp_GetKey();
	if (ucKeyCode > 0 && bsp_IsSelfCheckingReady())
	{
		/* �м����� */
		switch (ucKeyCode)
		{
			case KEY_1_DOWN:
			{
				bsp_KeySuspend();
			}break;
				
			case KEY_2_DOWN:
			{
				bsp_KeySuspend();
			}break;
				
			case KEY_3_DOWN:	
			{
				bsp_KeySuspend();
			}break;
			
			case KEY_1_UP:
			{
				
			}break;
				
			case KEY_2_UP:
			{

			}break;
				
			case KEY_3_UP:
			{

			}break;
			
			case KEY_1_LONG: /*�ػ�*/
			{
				if(xTaskGetTickCount() - bsp_GetLastKeyTick() >= PAUSE_INTERVAL_RESPONSE_TIME)
				{
					bsp_SetKeyRunLastState(RUN_STATE_SHUTDOWN);
					bsp_SperkerPlay(Song2);
					
					bsp_LedOff(LED_LOGO_CLEAN);
					bsp_LedOff(LED_LOGO_POWER);
					bsp_LedOff(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
					
					vTaskDelay(100);	
					while(bsp_SpeakerIsBusy()){}
					bsp_ClearKey();
				}
				
			}break;
			
			case KEY_2_LONG: /*���*/	
			{
				if(xTaskGetTickCount() - bsp_GetLastKeyTick() >= PAUSE_INTERVAL_RESPONSE_TIME)
				{
					bsp_SetKeyRunLastState(RUN_STATE_CHARGE);
					bsp_SperkerPlay(Song5);
					bsp_StartRunToggleLED(LED_LOGO_CHARGE);
					//bsp_StartCliffTest();
					bsp_StartSearchChargePile();
					
					vTaskDelay(200);	
					while(bsp_SpeakerIsBusy()){}
					bsp_ClearKey();
				}
				
			}break;
			
			case KEY_3_LONG: /*��ɨ*/
			{
				if(xTaskGetTickCount() - bsp_GetLastKeyTick() >= PAUSE_INTERVAL_RESPONSE_TIME)
				{
					bsp_SetKeyRunLastState(RUN_STATE_CLEAN);
					bsp_SperkerPlay(Song3);
					bsp_StartRunToggleLED(LED_LOGO_CLEAN);
					bsp_StartCliffTest();
					bsp_StartVacuum();
					
					vTaskDelay(200);	
					while(bsp_SpeakerIsBusy()){}
					bsp_ClearKey();
				}
				
			}break;
			
			case KEY_9_DOWN:
			{
				bsp_StopRunToggleLED();
				
				/*��λ��һ�εİ���״̬*/
				bsp_SetKeyRunLastState(RUN_STATE_DEFAULT);
				
				/*�رո���״̬��*/
				bsp_StopCliffTest();
				bsp_StopVacuum();
				/*�رյ��*/
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				bsp_StartEdgewiseRun();
				
			}break;
		}   
	}
}


/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
