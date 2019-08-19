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
	bsp_AngleRst();

#if 1
		bsp_SetMotorSpeed(MotorLeft,12);
		bsp_SetMotorSpeed(MotorRight,12);
#endif	

    while(1)
    {

		vTaskDelay(6000);
		
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
		
#if 0
		Collision collision = bsp_CollisionScan();
		
		if(collision == CollisionLeft)
		{
			//DEBUG("%06dLeft\r\n",index++);
			
			bsp_SetMotorSpeed(MotorLeft,-5);
			bsp_SetMotorSpeed(MotorRight,-12);
			vTaskDelay(500);
			bsp_SetMotorSpeed(MotorLeft,5);
			bsp_SetMotorSpeed(MotorRight,5);
		}
		else if(collision == CollisionRight)
		{
			//DEBUG("%06dRight\r\n",index++);
			
			bsp_SetMotorSpeed(MotorLeft,-12);
			bsp_SetMotorSpeed(MotorRight,-5);
			vTaskDelay(500);
			bsp_SetMotorSpeed(MotorLeft,5);
			bsp_SetMotorSpeed(MotorRight,5);
		}
		else if(collision == CollisionAll)
		{
			//DEBUG("%06dBoth\r\n",index++);
			
			bsp_SetMotorSpeed(MotorLeft,-12);
			bsp_SetMotorSpeed(MotorRight,-12);
			vTaskDelay(500);
			bsp_SetMotorSpeed(MotorLeft,5);
			bsp_SetMotorSpeed(MotorRight,5);
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
		
		bsp_PidSched(); /*10MS����һ��*/
		
//		DEBUG("L %d MM/S\r\n",bsp_MotorGetSpeed(MotorLeft));
//		DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
		
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
	
	
    while(1)
    {
		bsp_DetectAct();  /*����Թ���ѯɨ��*/
		bsp_DetectDeal(); /*����Թ�ɨ��������*/
		bsp_EdgewiseAct();/*�ر�*/
		
//		/*�ĸ�������չ�*/
//		bsp_GetCapCnt(CapCH1);
//		bsp_GetCapCnt(CapCH2);
//		bsp_GetCapCnt(CapCH3);
//		bsp_GetCapCnt(CapCH4);
//		
//		/*Ѱ�ҳ��׮*/
//		bsp_SearchChargingPileAct();
		
        vTaskDelay(1);
		
		
		int32_t leftCurrentSpeed  = 0 ;
		int32_t rightCurrentSpeed = 0 ;
		
		int32_t leftLastSpeed  = 0 ;
		int32_t rightLastSpeed = 0 ;
		
		double lastSpeed  = 0 ;
		double currentSpeed = 0 ;
		
		int32_t currentX = 0;
		int32_t currentY = 0;
		
		int32_t lastX = 0;
		int32_t lastY = 0;
		
		uint32_t currentTimestamp = 0;
		uint32_t lastTimestamp = 0;
		
		double last_orientation = 0;
		double current_orientation = 0;
		
		currentTimestamp = xTaskGetTickCount();
		{//per 20ms
			
			lastSpeed = currentSpeed;
			lastTimestamp = currentTimestamp; 
			last_orientation = current_orientation;
			lastX = currentX;
			lastY = currentY;
			
			currentTimestamp = xTaskGetTickCount();
			leftCurrentSpeed  = bsp_MotorGetSpeed(MotorLeft);
			rightCurrentSpeed = bsp_MotorGetSpeed(MotorRight);
			currentSpeed = (leftCurrentSpeed + rightCurrentSpeed) *0.5F*0.001 ; //m/s
			current_orientation = Deg2Rad(bsp_AngleReadRaw()*0.01f);
			
			
			
			currentX = lastX + (currentSpeed*cos(current_orientation)+lastSpeed*cos(last_orientation))*(currentTimestamp-lastTimestamp)*0.0005f;
			currentY = lastY + (currentSpeed*sin(current_orientation)+lastSpeed*sin(last_orientation))*(currentTimestamp-lastTimestamp)*0.0005f;
			
			
			
		}   
		
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
    xTaskCreate( vTaskTaskUserIF,   	/* ������  */
                 "vTaskUserIF",     	/* ������    */
                 1024,               	/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,              	/* �������  */
                 1,                 	/* �������ȼ�*/
                 &xHandleTaskUserIF );  /* ������  */
	
	
	xTaskCreate( vTaskLED,    		/* ������  */
                 "vTaskLED",  		/* ������    */
                 1024,         		/* stack��С����λword��Ҳ����4�ֽ� */
                 NULL,        		/* �������  */
                 2,           		/* �������ȼ�*/
                 &xHandleTaskLED ); /* ������  */
	
	xTaskCreate( vTaskMsgPro,     		/* ������  */
                 "vTaskMsgPro",   		/* ������    */
                 1024,             		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 3,               		/* �������ȼ�*/
                 &xHandleTaskMsgPro );  /* ������  */
	
	
	xTaskCreate( vTaskStart,     		/* ������  */
                 "vTaskStart",   		/* ������    */
                 1024,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 4,              		/* �������ȼ�*/
                 &xHandleTaskStart );   /* ������  */
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
