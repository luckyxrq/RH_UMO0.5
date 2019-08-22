#include "includes.h"


/*
**********************************************************************************************************
											函数声明
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
											变量声明
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskUserIF = NULL;
static TaskHandle_t xHandleTaskLED = NULL;
static TaskHandle_t xHandleTaskMsgPro = NULL;
static TaskHandle_t xHandleTaskStart = NULL;
static SemaphoreHandle_t  xMutex = NULL;



/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
int main(void)
{
	/* 
	  在启动调度前，为了防止初始化STM32外设时有中断服务程序执行，这里禁止全局中断(除了NMI和HardFault)。
	  这样做的好处是：
	  1. 防止执行的中断服务程序中有FreeRTOS的API函数。
	  2. 保证系统正常启动，不受别的中断影响。
	  3. 关于是否关闭全局中断，大家根据自己的实际情况设置即可。
	  在移植文件port.c中的函数prvStartFirstTask中会重新开启全局中断。通过指令cpsie i开启，__set_PRIMASK(1)
	  和cpsie i是等效的。
     */
	__set_PRIMASK(1);  
	
	/* 硬件初始化 */
	bsp_Init(); 
	
	/* 1. 初始化一个定时器中断，精度高于滴答定时器中断，这样才可以获得准确的系统信息 仅供调试目的，实际项
		  目中不要使用，因为这个功能比较影响系统实时性。
	   2. 为了正确获取FreeRTOS的调试信息，可以考虑将上面的关闭中断指令__set_PRIMASK(1); 注释掉。 
	*/
	vSetupSysInfoTest();
	
	/* 创建任务 */
	AppTaskCreate();

	/* 创建任务通信机制 */
	AppObjCreate();
	
    /* 启动调度，开始执行任务 */
    vTaskStartScheduler();

	/* 
	  如果系统正常启动是不会运行到这里的，运行到这里极有可能是用于定时器任务或者空闲任务的
	  heap空间不足造成创建失败，此要加大FreeRTOSConfig.h文件中定义的heap大小：
	  #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 17 * 1024 ) )
	*/
	while(1);
}

/*
*********************************************************************************************************
*	函 数 名: vTaskTaskUserIF
*	功能说明: 接口消息处理。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 1  (数值越小优先级越低，这个跟uCOS相反)
*********************************************************************************************************
*/
static void vTaskTaskUserIF(void *pvParameters)
{
	bsp_AngleRst();

#if 0
		bsp_SetMotorSpeed(MotorLeft,12);
		bsp_SetMotorSpeed(MotorRight,12);
#endif	

	
	
	
	
    while(1)
    {

		vTaskDelay(1000);
		DEBUG("2000ms tick\r\n");
		bsp_ParamUpdateTest();
		//bsp_StFlashTest(255);
	}
}

/*
*********************************************************************************************************
*	函 数 名: vTaskLED
*	功能说明: LED闪烁
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 2  
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
*	函 数 名: vTaskMsgPro
*	功能说明: 消息处理，这里用作DS18B20的温度采集和打印
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 3  
*********************************************************************************************************
*/
static void vTaskMsgPro(void *pvParameters)
{
    while(1)
    {
		//bsp_SendReportFrame();
		//bsp_PrintRemoteState();
		
		bsp_IWDG_Feed(); /* 喂狗 */
		
		bsp_PidSched(); /*10MS调用一次，这里面进行PWM计算，占空比设置，速度（脉冲为单位；MM为单位）计算*/
		
//		DEBUG("L %d MM/S\r\n",bsp_MotorGetSpeed(MotorLeft));
//		DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
		
		bsp_ComAnalysis();
		vTaskDelay(10);
    }
}

/*
*********************************************************************************************************
*	函 数 名: vTaskStart
*	功能说明: 启动任务，也就是最高优先级任务，这里用作按键扫描。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskStart(void *pvParameters)
{
	/*开启红外对管轮询扫描*/
	bsp_DetectStart(); 
//	/*开启寻找充电桩*/
//	bsp_StartSearchChargingPile();
	bsp_StartUpdatePos();
	
    while(1)
    {
		bsp_DetectAct();  /*红外对管轮询扫描*/
		bsp_DetectDeal(); /*红外对管扫描结果处理*/
//		bsp_EdgewiseAct();/*沿边*/
//		
//		/*四个红外接收管*/
//		bsp_GetCapCnt(CapCH1);
//		bsp_GetCapCnt(CapCH2);
//		bsp_GetCapCnt(CapCH3);
//		bsp_GetCapCnt(CapCH4);
//		
//		/*寻找充电桩*/
//		bsp_SearchChargingPileAct();
		/*更新坐标*/
		bsp_PositionUpdate();
		
        vTaskDelay(1);
		
		
		  
		
    }
}
				
/*
*********************************************************************************************************
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
    xTaskCreate( vTaskTaskUserIF,   	/* 任务函数  */
                 "vTaskUserIF",     	/* 任务名    */
                 1024,               	/* 任务栈大小，单位word，也就是4字节 */
                 NULL,              	/* 任务参数  */
                 1,                 	/* 任务优先级*/
                 &xHandleTaskUserIF );  /* 任务句柄  */
	
	
	xTaskCreate( vTaskLED,    		/* 任务函数  */
                 "vTaskLED",  		/* 任务名    */
                 1024,         		/* stack大小，单位word，也就是4字节 */
                 NULL,        		/* 任务参数  */
                 2,           		/* 任务优先级*/
                 &xHandleTaskLED ); /* 任务句柄  */
	
	xTaskCreate( vTaskMsgPro,     		/* 任务函数  */
                 "vTaskMsgPro",   		/* 任务名    */
                 1024,             		/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		/* 任务参数  */
                 3,               		/* 任务优先级*/
                 &xHandleTaskMsgPro );  /* 任务句柄  */
	
	
	xTaskCreate( vTaskStart,     		/* 任务函数  */
                 "vTaskStart",   		/* 任务名    */
                 1024,            		/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		/* 任务参数  */
                 4,              		/* 任务优先级*/
                 &xHandleTaskStart );   /* 任务句柄  */
}

/*
*********************************************************************************************************
*	函 数 名: AppObjCreate
*	功能说明: 创建任务通信机制
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppObjCreate (void)
{
	/* 创建互斥信号量 */
    xMutex = xSemaphoreCreateMutex();
	
	if(xMutex == NULL)
    {
        /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
    }
}

/*
*********************************************************************************************************
*	函 数 名: App_Printf
*	功能说明: 线程安全的printf方式		  			  
*	形    参: 同printf的参数。
*             在C中，当无法列出传递函数的所有实参的类型和数目时,可以用省略号指定参数表
*	返 回 值: 无
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

	/* 互斥信号量 */
	xSemaphoreTake(xMutex, portMAX_DELAY);

    printf("%s", buf_str);

   	xSemaphoreGive(xMutex);
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
