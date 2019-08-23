#include "includes.h"
/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
static void vTaskTaskUserIF(void *pvParameters);
static void vTaskLED(void *pvParameters);
static void vTaskMsgPro(void *pvParameters);
static void vTaskStart(void *pvParameters);

static void vTaskPerception(void *pvParameters);    //��֪ ��ȡ���������� ����Թܡ����¡���ײ����ء�������������л������������������
//static void vTaskPrediction(void *pvParameters);    //Ԥ�� ��̼ƹ���
//static void vTaskCommunication(void *pvParameters); //ͨ�� ����Э��֡ �ϴ�����������
static void vTaskDecision(void *pvParameters);      //���� ���������������
static void vTaskControl(void *pvParameters);       //���� ���ݾ��߿��Ƶ��



static void AppTaskCreate (void);
static void AppObjCreate (void);
void  App_Printf(char *format, ...);

/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskUserIF = NULL;
static TaskHandle_t xHandleTaskLED = NULL;
static TaskHandle_t xHandleTaskMsgPro = NULL;
static TaskHandle_t xHandleTaskStart = NULL;

static TaskHandle_t xHandleTaskPerception    = NULL;
//static TaskHandle_t xHandleTaskPrediction    = NULL;
//static TaskHandle_t xHandleTaskCommunication = NULL;
static TaskHandle_t xHandleTaskDecision      = NULL;
static TaskHandle_t xHandleTaskControl       = NULL;

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
*	�� �� ��: vTaskTaskUserIF
*	����˵��: �ӿ���Ϣ����
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 1  (��ֵԽС���ȼ�Խ�ͣ������uCOS�෴)
*********************************************************************************************************
*/
static void vTaskTaskUserIF(void *pvParameters)
{
	uint8_t ucKeyCode;	
	bsp_AngleRst();

#if 0
		bsp_SetMotorSpeed(MotorLeft,5);
		bsp_SetMotorSpeed(MotorRight,5);
#endif	
	
#if 0	
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-104));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-104));
#endif
	
    while(1)
    {
//		/* �������¼� */
//		ucKeyCode = bsp_GetKey();
//		if (ucKeyCode > 0)
//		{
//			/* �м����� */
//			switch (ucKeyCode)
//			{
//				case KEY_1_DOWN:/*����1����*/
//				{
//					
//				}break;
//				
//				case KEY_2_DOWN:/*����2����*/
//				{
//					
//				}break;
//				
//				case KEY_3_DOWN:/*����3����*/	
//				{
//					
//				}break;

//				case KEY_1_LONG:/*����1����*/	
//				{
//					
//				}break;
//				
//				case KEY_2_LONG:/*����2����*/	
//				{
//					
//				}break;
//				
//				case KEY_3_LONG:/*����3����*/	
//				{
//					
//				}break;
//			}
//		}
		vTaskDelay(20);
		
	}
}

/*
*********************************************************************************************************
*	�� �� ��: vTaskLED
*	����˵��: LED��˸
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2  
*********************************************************************************************************
*/
static void vTaskLED(void *pvParameters)
{
	//static uint32_t index = 0 ;
    while(1)
    {
		
#if 1
		Collision collision = bsp_CollisionScan();
		
		if(collision == CollisionLeft)
		{
			//DEBUG("%06dLeft\r\n",index++);
			
//			bsp_SetMotorSpeed(MotorLeft,-5);
//			bsp_SetMotorSpeed(MotorRight,-12);
//			vTaskDelay(500);
//			bsp_SetMotorSpeed(MotorLeft,5);
//			bsp_SetMotorSpeed(MotorRight,5);
		}
		else if(collision == CollisionRight)
		{
			//DEBUG("%06dRight\r\n",index++);
			
//			bsp_SetMotorSpeed(MotorLeft,-12);
//			bsp_SetMotorSpeed(MotorRight,-5);
//			vTaskDelay(500);
//			bsp_SetMotorSpeed(MotorLeft,5);
//			bsp_SetMotorSpeed(MotorRight,5);
		}
		else if(collision == CollisionAll)
		{
			//DEBUG("%06dBoth\r\n",index++);
			
//			bsp_SetMotorSpeed(MotorLeft,-12);
//			bsp_SetMotorSpeed(MotorRight,-12);
//			vTaskDelay(500);
//			bsp_SetMotorSpeed(MotorLeft,5);
//			bsp_SetMotorSpeed(MotorRight,5);
		}
#endif
		
		//bsp_ScopeSend();
		
		vTaskDelay(50);
    }
}

/*
*********************************************************************************************************
*	�� �� ��: vTaskMsgPro
*	����˵��: ��Ϣ������������DS18B20���¶Ȳɼ��ʹ�ӡ
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 3  
*********************************************************************************************************
*/
static void vTaskMsgPro(void *pvParameters)
{
    while(1)
    {
		//bsp_SendReportFrame();
		//bsp_PrintRemoteState();
		
		bsp_IWDG_Feed(); /* ι�� */
		
		bsp_PidSched(); /*10MS����һ�Σ����������PWM���㣬ռ�ձ����ã��ٶȣ�����Ϊ��λ��MMΪ��λ������*/
		
//		DEBUG("L %d MM/S\r\n",bsp_MotorGetSpeed(MotorLeft));
//		DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
		
		bsp_ComAnalysis();
		vTaskDelay(10);
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
static void vTaskStart(void *pvParameters)
{
	/*��������Թ���ѯɨ��*/
	bsp_DetectStart(); 
//	/*����Ѱ�ҳ��׮*/
//	bsp_StartSearchChargingPile();
	bsp_StartUpdatePos();
	
    while(1)
    {
		bsp_DetectAct();  /*����Թ���ѯɨ��*/
		bsp_DetectDeal(); /*����Թ�ɨ��������*/
//		bsp_EdgewiseAct();/*�ر�*/
//		
//		/*�ĸ�������չ�*/
//		bsp_GetCapCnt(CapCH1);
//		bsp_GetCapCnt(CapCH2);
//		bsp_GetCapCnt(CapCH3);
//		bsp_GetCapCnt(CapCH4);
//		
//		/*Ѱ�ҳ��׮*/
//		bsp_SearchChargingPileAct();
		/*��������*/
		bsp_PositionUpdate();
		
        vTaskDelay(1);
		
//		bsp_KeyScan();
			
    }
}








static void vTaskPerception(void *pvParameters)   //��֪ ��ȡ���������� ����Թܡ����¡���ײ����ء�������������л������������������
{
	bsp_AngleRst();
	
	bsp_DetectStart();  /*��������Թ���ѯɨ��*/
	bsp_StartUpdatePos();
	
	 while(1)
    {
		bsp_DetectAct();  /*����Թ���ѯɨ��*/
		bsp_DetectDeal(); /*����Թ�ɨ��������*/
		bsp_PositionUpdate(); /*��������*/
		bsp_KeyScan();	
		
        vTaskDelay(1);	
	}
	
}
static void vTaskControl(void *pvParameters)       //���� ���ݾ��߿��Ƶ��
{
	while(1)
    {

		bsp_IWDG_Feed(); /* ι�� */ //1S
		bsp_PidSched(); /*10MS����һ�Σ����������PWM���㣬ռ�ձ����ã��ٶȣ�����Ϊ��λ��MMΪ��λ������*/
	
		bsp_ComAnalysis();
		vTaskDelay(10);
    }
	
}
static void vTaskDecision(void *pvParameters)      //���� ���������������
{
	while(1)
	{
		Collision collision = bsp_CollisionScan();
		
		
		
		
		vTaskDelay(50);	
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
//    xTaskCreate( vTaskTaskUserIF,   	/* ������  */
//                 "vTaskUserIF",     	/* ������    */
//                 1024,               	/* ����ջ��С����λword��Ҳ����4�ֽ� */
//                 NULL,              	/* �������  */
//                 1,                 	/* �������ȼ�*/
//                 &xHandleTaskUserIF );  /* ������  */
//	
//	
//	xTaskCreate( vTaskLED,    		/* ������  */
//                 "vTaskLED",  		/* ������    */
//                 1024,         		/* stack��С����λword��Ҳ����4�ֽ� */
//                 NULL,        		/* �������  */
//                 2,           		/* �������ȼ�*/
//                 &xHandleTaskLED ); /* ������  */
//	
//	xTaskCreate( vTaskMsgPro,     		/* ������  */
//                 "vTaskMsgPro",   		/* ������    */
//                 1024,             		/* ����ջ��С����λword��Ҳ����4�ֽ� */
//                 NULL,           		/* �������  */
//                 3,               		/* �������ȼ�*/
//                 &xHandleTaskMsgPro );  /* ������  */
//	
//	
//	xTaskCreate( vTaskStart,     		/* ������  */
//                 "vTaskStart",   		/* ������    */
//                 1024,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
//                 NULL,           		/* �������  */
//                 4,              		/* �������ȼ�*/
//                 &xHandleTaskStart );   /* ������  */

	xTaskCreate( vTaskPerception,     		    /* ������  */
                 "vTaskPerception",   		    /* ������    */
                 1024,            		        /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 7,              		        /* �������ȼ�*/
                 &xHandleTaskPerception );      /* ������  */	
	xTaskCreate( vTaskControl,     		        /* ������  */
                 "vTaskControl",   		        /* ������    */
                 1024,            		        /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 6,              		        /* �������ȼ�*/
                 &xHandleTaskControl );         /* ������  */				 
	xTaskCreate( vTaskDecision,     		    /* ������  */
                 "vTaskDecision",   		    /* ������    */
                 1024,            		        /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		        /* �������  */
                 5,              		        /* �������ȼ�*/
                 &xHandleTaskDecision );        /* ������  */
				 
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
