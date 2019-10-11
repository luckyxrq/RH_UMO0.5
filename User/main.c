#include "includes.h"

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

/*
**********************************************************************************************************
                                            ��������
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskDecision      = NULL;
static TaskHandle_t xHandleTaskControl       = NULL;
static TaskHandle_t xHandleTaskPerception    = NULL;

static SemaphoreHandle_t  xMutex = NULL;


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
    uint8_t ucKeyCode;	
    uint32_t count = 0 ;
    static uint32_t keyFilterTime = 0 ;
	static uint32_t FilterTime = 1500 ;
	
    bsp_AngleRst();
	bsp_SperkerPlay(Song1);
	
    while(1)
    {
        /* �������¼� */
        ucKeyCode = bsp_GetKey();
        if (ucKeyCode > 0)
        {
            /* �м����� */
            switch (ucKeyCode)
            {
				case KEY_1_UP:/*����1����*/
				{
					if(bsp_IsLongPressedAgo(KEY_POWER))
					{
						bsp_SetIsLongPressedAgo(KEY_POWER , false);
					}
					else
					{
						if(xTaskGetTickCount() - keyFilterTime >= FilterTime)
						{
							DEBUG("����1�̰�\r\n");
							bsp_SetSuspendKey(true);
						}
						
					}
				}break;
					
				case KEY_2_UP:/*����2����*/
				{
					if(bsp_IsLongPressedAgo(KEY_CLEAN))
					{
						bsp_SetIsLongPressedAgo(KEY_CLEAN , false);
					}
					else
					{
						if(xTaskGetTickCount() - keyFilterTime >= FilterTime)
						{
							DEBUG("����2�̰�\r\n");
							bsp_SetSuspendKey(true);
						}
						
					}
				}break;
					
				case KEY_3_UP:/*����3����*/	
				{
					if(bsp_IsLongPressedAgo(KEY_CHARGE))
					{
						bsp_SetIsLongPressedAgo(KEY_CHARGE , false);
					}
					else
					{
						if(xTaskGetTickCount() - keyFilterTime >= FilterTime)
						{
							DEBUG("����3�̰�\r\n");
							bsp_SetSuspendKey(true);
						}
						
					}
				}break;
				
				case KEY_1_LONG:/*����1����*/	
				{
					bsp_SetIsLongPressedAgo(KEY_POWER , true);
					keyFilterTime = xTaskGetTickCount();
					
					DEBUG("����1����\r\n");
					//bsp_SperkerPlay(Song3);
					bsp_SetHomeKey(true);
					
				}break;
				
				case KEY_2_LONG:/*����2����*/	
				{
					bsp_SetIsLongPressedAgo(KEY_CLEAN , true);
					keyFilterTime = xTaskGetTickCount();
					
					DEBUG("����2����\r\n");
					//bsp_SperkerPlay(Song5);
					bsp_SetChargeKey(true);
				}break;
				
				case KEY_3_LONG:/*����3����*/	
				{
					bsp_SetIsLongPressedAgo(KEY_CHARGE , true);
					keyFilterTime = xTaskGetTickCount();
					
					DEBUG("����3����\r\n");
					//bsp_SperkerPlay(Song3);
					bsp_SetCleanKey(true);
				}break;

				
			}   
        }
        
		bsp_RunControl();   /* �������� */ 
		
        if(count++ % 2 == 0)
        {
//			bsp_PrintIR_Rev(); /*���ڴ�ӡ�������״̬*/
//			
//			float cliff = 0.0F;
//			
//			memset(&cliff,0,sizeof(cliff));
//			cliff = bsp_GetCliffVoltage(CliffLeft);
//			DEBUG("������ߣ�%.2F\r\n",cliff);
//			memset(&cliff,0,sizeof(cliff));
//			cliff = bsp_GetCliffVoltage(CliffMiddle);
//			DEBUG("�����м䣺%.2F\r\n",cliff);
//			memset(&cliff,0,sizeof(cliff));
//			cliff = bsp_GetCliffVoltage(CliffRight);
//			DEBUG("�����ұߣ�%.2F\r\n",cliff);
//			
        }

#if 0
		{
			static uint8_t songNum = 0 ;
			bsp_SperkerPlay(Song1 + songNum++);
			vTaskDelay(1000);
			while(bsp_SpeakerIsBusy())
			{
				vTaskDelay(1000);
			}
			if(songNum >= 20)
			{
				songNum = 0 ;
			}
		}
#endif

#if 0		
		{
			static uint32_t tick = 0 ;
			Collision ret = bsp_CollisionScan();
			if(ret == CollisionLeft)
			{
				DEBUG("%06d���<<<<<<<<<<\r\n",tick++);
			}
			else if(ret == CollisionRight)
			{
				DEBUG("%06d�ұ�>>>>>>>>>>\r\n",tick++);
			}
			else if(ret == CollisionAll)
			{
				DEBUG("%06d����==========\r\n",tick++);
			}
		}
#endif
		
#if 0
		bsp_StartVacuum();
		while(1)
		{
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,6);
			vTaskDelay(1500);
			bsp_SetMotorSpeed(MotorLeft, 12);
			bsp_SetMotorSpeed(MotorRight,12);
			vTaskDelay(1500);

			bsp_SetMotorSpeed(MotorLeft, -6);
			bsp_SetMotorSpeed(MotorRight,-6);
			vTaskDelay(1500); 
			bsp_SetMotorSpeed(MotorLeft, -12);
			bsp_SetMotorSpeed(MotorRight,-12);
			vTaskDelay(1500); 
		}
#endif

#if 0		
		DEBUG("L:%d  ",bsp_EncoderGetTotalMileage(EncoderLeft));
		DEBUG("R:%d\r\n",bsp_EncoderGetTotalMileage(EncoderRight));
#endif
		
        vTaskDelay(50);	
		
		
//		DEBUG("angle:%.2F\r\n",bsp_AngleRead());
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
	bsp_StopRunControl();    /*�رհ�������״̬��*/
	bsp_StartPowerOnToggle();/*��������˸����˸�ڼ�԰�����������Ӧ*/
	
    while(1)
    {
//        bsp_IWDG_Feed(); /* ι�� */
        
        bsp_PidSched(); /*10MS����һ�Σ����������PWM���㣬ռ�ձ����ã��ٶȣ�����Ϊ��λ��MMΪ��λ������*/
			
#if 0		
        DEBUG("L %d MM/S\r\n",bsp_MotorGetSpeed(MotorLeft));
        DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
#endif		
		
        bsp_ComAnalysis();
		
		bsp_PowerOnToggle();/* ����״̬�� */ 
		bsp_RunToggleLED();
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
	/*����Ѱ�ҳ��׮*/
	//bsp_StartSearchChargePile();
	/*�����ر�����*/
	//bsp_StartEdgewiseRun();
	/*����λ���������*/
    bsp_StartUpdatePos();
    /*����������ײЭ��*/
	bsp_StartAssistJudgeDirection();
	/*����դ���ͼ����*/
	
	
	bsp_StartUpdateGridMap();

	vTaskDelay(5000);
	
    while(1)
    {
#if 1
        bsp_DetectAct();  /*����Թ���ѯɨ��*/
        bsp_DetectDeal(); /*����Թ�ɨ��������*/
#endif
		
       
#if 0   /*���Ժ�����ľ��룬�⵽���ͣ����*/
		bsp_DetectMeasureTest();
#endif

#if 1   /*�������´�����*/		
		bsp_CliffTest();
#endif
		
        /*�ĸ�������չ�*/
#if 0 
        bsp_GetCapCnt(CapCH1);
        bsp_GetCapCnt(CapCH2);
        bsp_GetCapCnt(CapCH3);
        bsp_GetCapCnt(CapCH4);
#endif
        /*Ѱ�ҳ��׮*/
		bsp_SearchChargePile();
		/*�ر�����*/
		bsp_EdgewiseRun();
        /*��������*/
        bsp_PositionUpdate();
		
		/*���µ�ͼ*/
#if 0
		DEBUG("Start:%d\r\n",xTaskGetTickCount());
		bsp_GridMapUpdate(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(),bsp_CollisionScan(),bsp_GetIRSensorData());
		DEBUG("End:%d\r\n",xTaskGetTickCount());
#endif

		if(count++ % 5 == 0)
		{
			bsp_KeyScan();
			bsp_AssistJudgeDirection();
		}

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



/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
