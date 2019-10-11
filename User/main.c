#include "includes.h"

/*
**********************************************************************************************************
                                            函数声明
**********************************************************************************************************
*/
/*决策 整机软件控制流程*/
static void vTaskDecision(void *pvParameters);
/*控制 根据决策控制电机*/
static void vTaskControl(void *pvParameters);
/*感知 获取传感器数据 红外对管、跳崖、碰撞、离地、电机电流、尘盒霍尔、编码器、航向角*/
static void vTaskPerception(void *pvParameters);

static void AppTaskCreate (void);
static void AppObjCreate (void);
void  App_Printf(char *format, ...);

/*
**********************************************************************************************************
                                            变量声明
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskDecision      = NULL;
static TaskHandle_t xHandleTaskControl       = NULL;
static TaskHandle_t xHandleTaskPerception    = NULL;

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
*	函 数 名: vTaskStart
*	功能说明: 启动任务，也就是最高优先级任务，这里用作按键扫描。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskDecision(void *pvParameters)      //决策 整机软件控制流程
{
    uint8_t ucKeyCode;	
    uint32_t count = 0 ;
    static uint32_t keyFilterTime = 0 ;
	static uint32_t FilterTime = 1500 ;
	
    bsp_AngleRst();
	bsp_SperkerPlay(Song1);
	
    while(1)
    {
        /* 处理按键事件 */
        ucKeyCode = bsp_GetKey();
        if (ucKeyCode > 0)
        {
            /* 有键按下 */
            switch (ucKeyCode)
            {
				case KEY_1_UP:/*按键1按下*/
				{
					if(bsp_IsLongPressedAgo(KEY_POWER))
					{
						bsp_SetIsLongPressedAgo(KEY_POWER , false);
					}
					else
					{
						if(xTaskGetTickCount() - keyFilterTime >= FilterTime)
						{
							DEBUG("按键1短按\r\n");
							bsp_SetSuspendKey(true);
						}
						
					}
				}break;
					
				case KEY_2_UP:/*按键2按下*/
				{
					if(bsp_IsLongPressedAgo(KEY_CLEAN))
					{
						bsp_SetIsLongPressedAgo(KEY_CLEAN , false);
					}
					else
					{
						if(xTaskGetTickCount() - keyFilterTime >= FilterTime)
						{
							DEBUG("按键2短按\r\n");
							bsp_SetSuspendKey(true);
						}
						
					}
				}break;
					
				case KEY_3_UP:/*按键3按下*/	
				{
					if(bsp_IsLongPressedAgo(KEY_CHARGE))
					{
						bsp_SetIsLongPressedAgo(KEY_CHARGE , false);
					}
					else
					{
						if(xTaskGetTickCount() - keyFilterTime >= FilterTime)
						{
							DEBUG("按键3短按\r\n");
							bsp_SetSuspendKey(true);
						}
						
					}
				}break;
				
				case KEY_1_LONG:/*按键1长按*/	
				{
					bsp_SetIsLongPressedAgo(KEY_POWER , true);
					keyFilterTime = xTaskGetTickCount();
					
					DEBUG("按键1长按\r\n");
					//bsp_SperkerPlay(Song3);
					bsp_SetHomeKey(true);
					
				}break;
				
				case KEY_2_LONG:/*按键2长按*/	
				{
					bsp_SetIsLongPressedAgo(KEY_CLEAN , true);
					keyFilterTime = xTaskGetTickCount();
					
					DEBUG("按键2长按\r\n");
					//bsp_SperkerPlay(Song5);
					bsp_SetChargeKey(true);
				}break;
				
				case KEY_3_LONG:/*按键3长按*/	
				{
					bsp_SetIsLongPressedAgo(KEY_CHARGE , true);
					keyFilterTime = xTaskGetTickCount();
					
					DEBUG("按键3长按\r\n");
					//bsp_SperkerPlay(Song3);
					bsp_SetCleanKey(true);
				}break;

				
			}   
        }
        
		bsp_RunControl();   /* 整机控制 */ 
		
        if(count++ % 2 == 0)
        {
//			bsp_PrintIR_Rev(); /*用于打印红外接收状态*/
//			
//			float cliff = 0.0F;
//			
//			memset(&cliff,0,sizeof(cliff));
//			cliff = bsp_GetCliffVoltage(CliffLeft);
//			DEBUG("跳崖左边：%.2F\r\n",cliff);
//			memset(&cliff,0,sizeof(cliff));
//			cliff = bsp_GetCliffVoltage(CliffMiddle);
//			DEBUG("跳崖中间：%.2F\r\n",cliff);
//			memset(&cliff,0,sizeof(cliff));
//			cliff = bsp_GetCliffVoltage(CliffRight);
//			DEBUG("跳崖右边：%.2F\r\n",cliff);
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
				DEBUG("%06d左边<<<<<<<<<<\r\n",tick++);
			}
			else if(ret == CollisionRight)
			{
				DEBUG("%06d右边>>>>>>>>>>\r\n",tick++);
			}
			else if(ret == CollisionAll)
			{
				DEBUG("%06d两边==========\r\n",tick++);
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
*	函 数 名: vTaskStart
*	功能说明: 启动任务，也就是最高优先级任务，这里用作按键扫描。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskControl(void *pvParameters)       //控制 根据决策控制电机
{
	bsp_StopRunControl();    /*关闭按键控制状态机*/
	bsp_StartPowerOnToggle();/*开机先闪烁，闪烁期间对按键操作不响应*/
	
    while(1)
    {
//        bsp_IWDG_Feed(); /* 喂狗 */
        
        bsp_PidSched(); /*10MS调用一次，这里面进行PWM计算，占空比设置，速度（脉冲为单位；MM为单位）计算*/
			
#if 0		
        DEBUG("L %d MM/S\r\n",bsp_MotorGetSpeed(MotorLeft));
        DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
#endif		
		
        bsp_ComAnalysis();
		
		bsp_PowerOnToggle();/* 开机状态灯 */ 
		bsp_RunToggleLED();
        vTaskDelay(10);
    }
    
}


/*
*********************************************************************************************************
*	函 数 名: vTaskStart
*	功能说明: 感知 获取传感器数据 红外对管、跳崖、碰撞、离地、电机电流、尘盒霍尔、编码器、航向角。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskPerception(void *pvParameters)
{
	uint32_t count = 0 ;
	
    /*开启红外对管轮询扫描*/
    bsp_DetectStart(); 
	/*开启寻找充电桩*/
	//bsp_StartSearchChargePile();
	/*开启沿边行走*/
	//bsp_StartEdgewiseRun();
	/*开启位置坐标更新*/
    bsp_StartUpdatePos();
    /*开启正面碰撞协助*/
	bsp_StartAssistJudgeDirection();
	/*开启栅格地图跟新*/
	
	
	bsp_StartUpdateGridMap();

	vTaskDelay(5000);
	
    while(1)
    {
#if 1
        bsp_DetectAct();  /*红外对管轮询扫描*/
        bsp_DetectDeal(); /*红外对管扫描结果处理*/
#endif
		
       
#if 0   /*测试红外测距的距离，测到后就停下来*/
		bsp_DetectMeasureTest();
#endif

#if 1   /*测试跳崖传感器*/		
		bsp_CliffTest();
#endif
		
        /*四个红外接收管*/
#if 0 
        bsp_GetCapCnt(CapCH1);
        bsp_GetCapCnt(CapCH2);
        bsp_GetCapCnt(CapCH3);
        bsp_GetCapCnt(CapCH4);
#endif
        /*寻找充电桩*/
		bsp_SearchChargePile();
		/*沿边行走*/
		bsp_EdgewiseRun();
        /*更新坐标*/
        bsp_PositionUpdate();
		
		/*更新地图*/
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
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
    xTaskCreate( vTaskDecision,     		    /* 任务函数  */
                 "vTaskDecision",   		    /* 任务名    */
                 1024,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 1,              		        /* 任务优先级*/
                 &xHandleTaskDecision );        /* 任务句柄  */
    xTaskCreate( vTaskControl,     		        /* 任务函数  */
                 "vTaskControl",   		        /* 任务名    */
                 1024,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 2,              		        /* 任务优先级*/
                 &xHandleTaskControl );         /* 任务句柄  */	
    xTaskCreate( vTaskPerception,     		    /* 任务函数  */
                 "vTaskPerception",   		    /* 任务名    */
                 1024,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 3,              		        /* 任务优先级*/
                 &xHandleTaskPerception );      /* 任务句柄  */	
    
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
