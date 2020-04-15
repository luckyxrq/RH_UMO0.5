#include "includes.h"

/*
**********************************************************************************************************
                                            宏定义
**********************************************************************************************************
*/
#define PAUSE_INTERVAL_RESPONSE_TIME         400
#define AT_POWER_ON_OPEN_ALL_MODULE_EN       0     /*在开机的时候直接打开所有的电机轮子...，用于调试的时候使用*/

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
static void bsp_KeySuspend(void);
static void bsp_KeyProc(void);
/*
**********************************************************************************************************
                                            变量声明
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskDecision      = NULL;
static TaskHandle_t xHandleTaskControl       = NULL;
static TaskHandle_t xHandleTaskPerception    = NULL;

static SemaphoreHandle_t  xMutex = NULL;
static KeyProc keyProc;
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
    
    uint32_t count = 0 ;
    
	
    bsp_AngleRst();
	bsp_SperkerPlay(Song1);
	
    while(1)
    {
        /* 处理按键事件 */
        bsp_KeyProc();
		
		
        if(count++ % 10 == 0)
        {
#if 0 
			bsp_PrintIR_Rev(); /*用于打印红外接收状态*/
#endif
			bsp_ChangeWifi2SmartConfigStateProc();
			
			/*下面是打印开关，酌情注释*/
			bsp_WifiStateProc();
			//bsp_PrintCollision();
			//bsp_PrintIR_Rev();
        }
		
#if 1 /*更新地图*/
		
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
*	函 数 名: vTaskStart
*	功能说明: 启动任务，也就是最高优先级任务，这里用作按键扫描。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskControl(void *pvParameters)       //控制 根据决策控制电机
{
	uint32_t count = 0 ;
	
    while(1)
    {
#if 0
        bsp_IWDG_Feed(); /* 喂狗 */
#endif
        		
#if 0		
        DEBUG("L %d MM/S\r\n",bsp_MotorGetSpeed(MotorLeft));
        DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
#endif		
		
		
        bsp_ComAnalysis();
		bsp_PowerOnToggle();/* 开机状态灯 */ 
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
	
	/*检测主机悬空*/
	//bsp_StartOffSiteProc();
	
	/*开启寻找充电桩*/
	//bsp_StartSearchChargePile();
	
	/*开启沿边行走*/
	//bsp_StartEdgewiseRun();
	
	/*开启位置坐标更新*/
    bsp_StartUpdatePos();
	
    /*开启正面碰撞协助*/
	//bsp_StartAssistJudgeDirection();
	
	/*开启栅格地图跟新*/
	//bsp_StartUpdateGridMap();

	/*开清扫策略*/
	//bsp_StartUpdateCleanStrategyB();
	
	//bsp_StartCliffTest();

	

	if( !bsp_IsSelfCheckingReady())
	{
		 vTaskDelay(1);	
	}	
	
	
	bsp_InitCliffSW();
	
#if AT_POWER_ON_OPEN_ALL_MODULE_EN /*在开机的时候直接打开所有的电机轮子...，用于调试的时候使用*/
	bsp_StartVacuum();
	bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
#endif
	
	
    while(1)
    {
#if 1
        bsp_DetectAct();  /*红外对管轮询扫描*/
        bsp_DetectDeal(); /*红外对管扫描结果处理*/
#endif
		
       
#if 0   /*测试红外测距的距离，测到后就停下来*/
		bsp_DetectMeasureTest();
#endif

#if 0   /*测试跳崖传感器 、红外、碰撞共同测试*/	 	
		bsp_CliffTest();
#endif
		
	
		/*检测主机悬空*/
		bsp_OffSiteProc();
        /*寻找充电桩*/
		bsp_SearchChargePile();
		/*沿边行走*/
		bsp_EdgewiseRun();
        /*更新坐标*/
        bsp_PositionUpdate();
		

		if(count % 10 == 0)
		{
			bsp_PidSched(); /*10MS调用一次，这里面进行PWM计算，占空比设置，速度（脉冲为单位；MM为单位）计算*/
			//bsp_AssistJudgeDirection();
		}
		
		
		
		wifi_uart_service();
		
		count++;
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

/*
*********************************************************************************************************
*	函 数 名: bsp_KeySuspend
*	功能说明: 不同状态暂停键的效果不同	  			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_KeySuspend(void)
{
	RunState lastRunState = bsp_GetKeyRunLastState();
	
	if(lastRunState != RUN_STATE_DEFAULT)
	{
		/*记录下短按的时间*/
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
*	函 数 名: bsp_SetKeyRunLastState
*	功能说明: 设置上次的按键状态	  			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetKeyRunLastState(RunState state)
{
	keyProc.lastRunState = state;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetKeyRunLastState
*	功能说明: 获取上次的按键状态	  			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
RunState bsp_GetKeyRunLastState(void)
{
	return keyProc.lastRunState;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetLastKeyTick
*	功能说明: 记录上次按键的时间 			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetLastKeyTick(uint32_t tick)
{
	keyProc.lastKeyTick = tick;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetLastKeyTick
*	功能说明: 获取上次按键的时间 			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint32_t bsp_GetLastKeyTick(void)
{
	return keyProc.lastKeyTick;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_KeyProc
*	功能说明: 按键处理函数	  			  
*	形    参: 无
*	返 回 值: 无
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
//		/* 有键按下 */
//		switch (ucKeyCode)
//		{
//			case KEY_DOWN_POWER:
//			{
//				DEBUG("电源按键按下\r\n");
//				bsp_KeySuspend();
//			}break;
//				
//			case KEY_DOWN_CHARGE:
//			{
//				DEBUG("充电按键按下\r\n");
//				bsp_KeySuspend();
//			}break;
//				
//			case KEY_DOWN_CLEAN:	
//			{
//				DEBUG("清扫按键按下\r\n");
//				bsp_KeySuspend();
//			}break;
//			

//			
//			case KEY_LONG_POWER: /*关机*/
//			{
//				DEBUG("电源按键长按\r\n");
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
//			case KEY_LONG_CHARGE: /*充电*/	
//			{
//				DEBUG("充电按键长按\r\n");
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
//			case KEY_LONG_CLEAN: /*清扫*/
//			{
//				DEBUG("清扫按键长按\r\n");
//				if(xTaskGetTickCount() - bsp_GetLastKeyTick() >= PAUSE_INTERVAL_RESPONSE_TIME)
//				{
//					bsp_SetKeyRunLastState(RUN_STATE_CLEAN);
//					bsp_SperkerPlay(Song3);
//					bsp_StartRunToggleLED(LED_LOGO_CLEAN);
//					
//					//bsp_StartCliffTest();
//					/*开清扫策略*/
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
//				/*复位上一次的按键状态*/
//				bsp_SetKeyRunLastState(RUN_STATE_DEFAULT);
//				
//				
//				
//				/*关闭各种状态机*/
//				bsp_StopCliffTest();
//				bsp_StopVacuum();
//				/*关闭电机*/
//				bsp_SetMotorSpeed(MotorLeft, 0);
//				bsp_SetMotorSpeed(MotorRight,0);
//				bsp_StartEdgewiseRun();
//				
//			}break;
//			
//			
//			case KEY_10_DOWN:
//			{
//				DEBUG("重新配网：同时按充电和清扫\r\n");
//				bsp_SperkerPlay(Song29);
//				bsp_StartChangeWifi2SmartConfigState();
//				bsp_StartRunToggleLED(LED_WIFI_LINK);
//			}break;
//		}   
//	}
}



/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
